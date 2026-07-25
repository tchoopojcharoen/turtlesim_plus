#!/usr/bin/python3

import math
import random

import py_trees
import py_trees_ros.subscribers as subscribers
import rclpy.qos
import std_srvs.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.msg import ScannerDataArray


class PizzaDetected(subscribers.ToBlackboard):
    """Subscribes to /[turtle]/scan and derives the nearest currently-visible
    pizza's bearing/distance onto the blackboard as `nearest_pizza`.

    Doubles as a condition: SUCCESS if a pizza is currently in view, FAILURE
    otherwise. /[turtle]/scan only publishes when something is within the
    scanner's cone -- it never publishes an empty message -- so "in view" also
    requires the last received message to be recent; a stale message is treated
    as nothing detected rather than as a stuck last-known position.
    """

    def __init__(self, name: str, topic_name: str,
                 qos_profile: rclpy.qos.QoSProfile,
                 staleness_threshold: float = 0.4):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=ScannerDataArray,
            qos_profile=qos_profile,
            blackboard_variables={"scan": None},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER,
        )
        self.blackboard.register_key(key="nearest_pizza", access=py_trees.common.Access.WRITE)
        self.blackboard.nearest_pizza = None
        self.staleness_threshold = staleness_threshold
        self._last_received = None

    def _callback(self, msg):
        super()._callback(msg)
        self._last_received = self.node.get_clock().now()

    def update(self) -> py_trees.common.Status:
        super().update()
        fresh = (
            self._last_received is not None
            and (self.node.get_clock().now() - self._last_received).nanoseconds / 1e9 <= self.staleness_threshold
        )
        pizza = None
        if fresh:
            pizzas = [d for d in self.blackboard.scan.data if d.type == 'Pizza']
            if pizzas:
                nearest = min(pizzas, key=lambda d: d.distance)
                pizza = {'angle': nearest.angle, 'distance': nearest.distance}
        self.blackboard.nearest_pizza = pizza
        return py_trees.common.Status.SUCCESS if pizza is not None else py_trees.common.Status.FAILURE


class PizzaInEatRange(py_trees.behaviour.Behaviour):
    """SUCCESS if the nearest known pizza is within eating distance."""

    def __init__(self, name: str, eat_range_margin: float = 1.8):
        super().__init__(name=name)
        self.eat_range_margin = eat_range_margin
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="nearest_pizza", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        pizza = self.blackboard.nearest_pizza
        if pizza is not None and pizza['distance'] <= self.eat_range_margin:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class CallEatService(py_trees.behaviour.Behaviour):
    """Calls /[turtle]/eat (std_srvs/Empty) once per activation and waits for
    the response. There's no py_trees_ros helper for service clients (only
    subscribers/publishers/action-clients ship in this version), so this is a
    minimal hand-rolled rclpy client following the same setup/initialise/update
    lifecycle used by py_trees_ros's own action-client behaviours."""

    def __init__(self, name: str, turtle_name: str):
        super().__init__(name=name)
        self.turtle_name = turtle_name
        self.node = None
        self.client = None
        self.future = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(f"didn't find 'node' in setup's kwargs [{self.name}]") from e
        self.client = self.node.create_client(std_srvs.srv.Empty, f'/{self.turtle_name}/eat')

    def initialise(self):
        self.future = self.client.call_async(std_srvs.srv.Empty.Request())

    def update(self) -> py_trees.common.Status:
        if self.future is None or not self.future.done():
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.future = None


class DriveTowardPizza(py_trees.behaviour.Behaviour):
    """Publishes cmd_vel to turn toward and approach the nearest known pizza."""

    def __init__(self, name: str, turtle_name: str, k_ang: float = 5.0, linear_speed: float = 2.0,
                 cone_half_angle: float = math.pi / 6):
        super().__init__(name=name)
        self.turtle_name = turtle_name
        self.k_ang = k_ang
        self.linear_speed = linear_speed
        self.cone_half_angle = cone_half_angle
        self.node = None
        self.publisher = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="nearest_pizza", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(f"didn't find 'node' in setup's kwargs [{self.name}]") from e
        self.publisher = self.node.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)

    def update(self) -> py_trees.common.Status:
        pizza = self.blackboard.nearest_pizza
        if pizza is None:
            return py_trees.common.Status.FAILURE
        angle = pizza['angle']
        msg = Twist()
        msg.angular.z = self.k_ang * angle
        # Ease off forward speed while still turning to face the pizza -- full
        # speed once roughly aligned, ~0 at the cone's edge -- so it turns to
        # face the target first instead of tracing a wide arc at full speed
        # the whole time (which also made the final alignment look sluggish,
        # since a bigger angular error meant it was also further off-course
        # positionally by the time it turned in).
        alignment = max(0.0, 1.0 - abs(angle) / self.cone_half_angle)
        msg.linear.x = self.linear_speed * alignment
        self.publisher.publish(msg)
        return py_trees.common.Status.RUNNING


class Wander(py_trees.behaviour.Behaviour):
    """Fallback while no pizza is in view: spins in place, tracking cumulative
    rotation measured from actual pose feedback (not open-loop time*speed, so
    it stays correct regardless of tick timing). Once a full revolution has
    passed without detecting food, it picks a random point in the world and
    drives there, then resumes spinning to scan the new spot -- repeating
    indefinitely so the turtle keeps covering new ground instead of orbiting
    forever in place. Interrupted immediately (via terminate/re-initialise)
    whenever a pizza is spotted, since the parent Selector re-evaluates
    PizzaDetected before this behaviour every tick.
    """

    def __init__(self, name: str, turtle_name: str, spin_speed: float = 1.6,
                 linear_speed: float = 2.0, k_ang: float = 2.0,
                 arrival_tolerance: float = 0.3, world_size: float = 10.88):
        super().__init__(name=name)
        self.turtle_name = turtle_name
        self.spin_speed = spin_speed
        self.linear_speed = linear_speed
        self.k_ang = k_ang
        self.arrival_tolerance = arrival_tolerance
        self.world_size = world_size
        self.node = None
        self.publisher = None
        self.pose = None
        self._mode = 'spin'
        self._spun_radians = 0.0
        self._last_theta = None
        self._target = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(f"didn't find 'node' in setup's kwargs [{self.name}]") from e
        self.publisher = self.node.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)
        self.node.create_subscription(Pose, f'/{self.turtle_name}/pose', self._pose_callback, 10)

    def _pose_callback(self, msg: Pose):
        self.pose = msg

    def initialise(self):
        # (re)entering Wander after Chase & Eat succeeded elsewhere (or on first
        # activation) -- start a fresh spin-and-scan cycle.
        self._mode = 'spin'
        self._spun_radians = 0.0
        self._last_theta = self.pose.theta if self.pose is not None else None
        self._target = None

    @staticmethod
    def _normalize(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def update(self) -> py_trees.common.Status:
        if self.pose is None:
            return py_trees.common.Status.RUNNING

        if self._mode == 'spin':
            if self._last_theta is not None:
                self._spun_radians += abs(self._normalize(self.pose.theta - self._last_theta))
            self._last_theta = self.pose.theta
            if self._spun_radians >= 2 * math.pi:
                # completed a full revolution without seeing food -- relocate.
                self._mode = 'walk'
                self._target = (
                    random.uniform(0.0, self.world_size),
                    random.uniform(0.0, self.world_size),
                )
            else:
                msg = Twist()
                msg.angular.z = self.spin_speed
                self.publisher.publish(msg)
                return py_trees.common.Status.RUNNING

        # mode == 'walk'
        dx = self._target[0] - self.pose.x
        dy = self._target[1] - self.pose.y
        distance = math.hypot(dx, dy)
        if distance <= self.arrival_tolerance:
            self._mode = 'spin'
            self._spun_radians = 0.0
            self._last_theta = self.pose.theta
            self._target = None
            return py_trees.common.Status.RUNNING

        bearing = self._normalize(math.atan2(dy, dx) - self.pose.theta)
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.k_ang * bearing
        self.publisher.publish(msg)
        return py_trees.common.Status.RUNNING
