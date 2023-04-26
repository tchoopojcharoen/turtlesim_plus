#!/usr/bin/python3

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# The GPL3 license is a widely used free software license that allows 
# users to run, study, share, and modify software. It requires that 
# any derivative works or modifications made to the software be 
# licensed under the GPL3 and that the source code be made available 
# to anyone who receives the software.


# other libraries
import os
import random
import abc
from typing import Dict
import pygame

# package module
from turtlesim_plus.entity import (
    Entity, Pizza, Turtle, 
    TurtleCommandInterface,TurtleScannerInterface,TurtleEatInterface, 
    Simulator
)

# RCLPY libraries, classes, functions
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

# ROS Packages
from std_msgs.msg import Int64
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist, Point
from turtlesim_plus_interfaces.msg import ScannerData, ScannerDataArray
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim_plus_interfaces.action import GetData

class ROS2Plugin(abc.ABC):
    def __init__(self,node:Node):
        self.node = node
class MouseROS2Plugin(ROS2Plugin):
    def __init__(self,node:Node):
        super().__init__(node=node)
        self.mouse_pub = self.node.create_publisher(Point,'mouse_position',10)
        self.node.simulator.mouse_callback = self.mouse_callback
    def mouse_callback(self,pos):
        msg = Point()
        msg.x = pos[0]*5.44/250
        msg.y = 10.88-pos[1]*5.44/250
        self.mouse_pub.publish(msg)
class TurtleCommandROS2Plugin(TurtleCommandInterface,ROS2Plugin):
    def __init__(self, turtle: Turtle,node: Node,image):
        TurtleCommandInterface.__init__(self,turtle=turtle)
        ROS2Plugin.__init__(self,node=node)
        self.cmd_vel_pub = self.node.create_publisher(Pose,self.turtle.name+'/pose',10)
        self.sub = self.node.create_subscription(Twist,self.turtle.name+'/cmd_vel',self.cmd_vel_sub_callback,10)
        self.stop_service = self.node.create_service(Empty,self.turtle.name+'/stop',self.stop_srv_callback)
        self.image = image
    def cmd_vel_sub_callback(self,msg:Twist):
        self.command_velocity = [msg.linear.x,msg.angular.z]
    def stop_srv_callback(self,request:Empty.Request,response:Empty.Response):
        self.command_velocity = [0.0,0.0]
        return response
    def update(self, dt: float = 0.1, *args, **kwargs):
        TurtleCommandInterface.update(self, dt, *args, **kwargs)
        self.set_state(self.turtle.state)
        self.set_pose(self.turtle.state)
        msg = Pose()
        msg.x = self.state[0]
        msg.y = self.state[1]
        msg.theta = self.state[2]
        #qw = np.cos(yaw/2) 
        #qz = np.sin(yaw/2) 
        self.cmd_vel_pub.publish(msg=msg)
    def __del__(self):
        self.node.destroy_publisher(publisher=self.cmd_vel_pub)
        self.node.destroy_subscription(subscription=self.sub)
        self.node.destroy_service(service=self.stop_service)
class TurtleScannerROS2Plugin(TurtleScannerInterface,ROS2Plugin):
    def __init__(self, turtle:Turtle,node:Node):
        TurtleScannerInterface.__init__(self,turtle=turtle)
        ROS2Plugin.__init__(self,node=node)
        self.scanner_pub = self.node.create_publisher(ScannerDataArray,self.turtle.name+'/scan',10)
        self.action_server = ActionServer(self.node,GetData,self.turtle.name+'/detect_pizza',self.execute_callback)
    def execute_callback(self,goal_handle:ServerGoalHandle):
        result = GetData.Result()
        if len(self.scanner_output)>0:
            result.is_data = True
            msg = ScannerDataArray()
            for data in self.scanner_output:
                scanner_data = ScannerData()
                if str(data.type.__name__)=='Pizza':
                    scanner_data.type = 'Pizza'
                    scanner_data.angle = data.angle
                    scanner_data.distance = data.distance
                    msg.data.append(scanner_data)
            result.data = msg
            goal_handle.succeed()
        else:
            result.is_data = False
            goal_handle.abort()
        return result
    def update(self, dt: float = 0.1, entity_list:Dict[str,Entity] = {}, *args, **kwargs):
        TurtleScannerInterface.update(self,dt=dt,entity_list=entity_list)
        self.set_state(self.turtle.state)
        if len(self.scanner_output)>0:
            msg = ScannerDataArray()
            for data in self.scanner_output:
                scanner_data = ScannerData()
                if str(data.type.__name__).startswith('Turtle'):
                    scanner_data.type = 'Turtle'
                else:
                    scanner_data.type = data.type.__name__
                scanner_data.angle = data.angle
                scanner_data.distance = data.distance
                msg.data.append(scanner_data)
            self.scanner_pub.publish(msg)
    def __del__(self):
        self.node.destroy_publisher(publisher=self.scanner_pub)
class TurtleEatROS2Plugin(TurtleEatInterface,ROS2Plugin):
    def __init__(self, turtle:Turtle,node:Node):
        TurtleEatInterface.__init__(self,turtle=turtle)
        ROS2Plugin.__init__(self,node=node)
        self.pizza_count = 0
        self.pizza_count_publisher = self.node.create_publisher(Int64,self.turtle.name+'/pizza_count',10)
        self.eat_service = self.node.create_service(Empty,self.turtle.name+'/eat',self.eat_srv_callback)
    def eat_srv_callback(self,request:Empty.Request,response:Empty.Response):
        if len(self.edibles)>0:
            self.pizza_count+=1
            edible = self.edibles.pop(0)
            del self.node.simulator.entity_list[edible.name]
            del self.node.simulator.gui.entity_list[edible.name]
        return response
    def update(self, dt: float = 0.1, entity_list:Dict[str,Entity] = {}, *args, **kwargs):
        TurtleEatInterface.update(self,dt=dt,entity_list=entity_list)
        self.set_state(self.turtle.state)
        msg = Int64()
        msg.data = self.pizza_count
        self.pizza_count_publisher.publish(msg)
    def __del__(self):
        self.node.destroy_publisher(publisher=self.pizza_count_publisher)
        self.node.destroy_service(service=self.eat_service)

class TurtleCommandScannerEatROS2Plugin(TurtleScannerROS2Plugin,TurtleCommandROS2Plugin,TurtleEatROS2Plugin):
    def __init__(self, turtle: Turtle, node: Node, image):
        TurtleCommandROS2Plugin.__init__(self,turtle=turtle, node=node, image=image)
        TurtleScannerROS2Plugin.__init__(self,turtle=turtle,node=node)
        TurtleEatROS2Plugin.__init__(self,turtle=turtle,node=node)
    def update(self, dt: float = 0.1, entity_list: Dict[str,Entity] = {}, *args, **kwargs):
        TurtleCommandROS2Plugin.update(self,dt=dt)
        TurtleScannerROS2Plugin.update(self,dt=dt,entity_list=entity_list)
        TurtleEatROS2Plugin.update(self,dt=dt,entity_list=entity_list)
    def render(self, screen):
        TurtleScannerROS2Plugin.render(self,screen=screen)
        TurtleEatROS2Plugin.render(self,screen=screen)
        TurtleCommandROS2Plugin.render(self,screen=screen)
    def __del__(self):
        TurtleEatROS2Plugin.__del__(self)
        TurtleScannerROS2Plugin.__del__(self)
        TurtleCommandROS2Plugin.__del__(self)
class TurtlesimPlusNode(Node):
    def __init__(self):
        super().__init__(node_name='turtlesim_plus')
        time_step = 0.01 
        self.simulator = Simulator(time_step=time_step)
        self.mouse_plugin = MouseROS2Plugin(self)
        self.create_timer(time_step,self.timer_callback)
        self.spawn_turtle_service = self.create_service(Spawn,'spawn_turtle',self.spawn_turtle_srv_callback)
        self.remove_turtle_service = self.create_service(Kill,'remove_turtle',self.remove_turtle_srv_callback)
        self.spawn_pizza_service = self.create_service(GivePosition,'spawn_pizza',self.spawn_pizza_srv_callback)
        prompt = """
        
        Welcome to Turtlesim+!!

        You can call the following services to interact with 'the simulator':
        /spawn_turtle,/remove_turtle,/spawn_pizza

        "Once you spawn at least 1 turtle, you can read from the following topics:
        /[name]/pose,/[name]/scan,/[name]/pizza_count

        You can also publish to '/[name]/cmd_vel'.

        You can also call the following turtle's services.
        /[name]/eat,/[name]/stop

        """
        self.get_logger().info(prompt)
        
        request = Spawn.Request()
        request.name = 'turtle1'
        response = Spawn.Response()
        self.spawn_turtle_srv_callback(request,response)
        
        
    def spawn_turtle_srv_callback(self,request:Spawn.Request,response:Spawn.Response):
        flag = False
        init_pose = [5.44,5.44,0.0]
        if not request.name:
            name = 'turtle1'
            flag = True
        else:
            name = request.name
        idx = 0
        while name in self.simulator.entity_list.keys():
            idx += 1
            name = 'turtle'+str(idx)
        if flag:
            self.get_logger().info(f'Name is not given. Use {name} instead.')
        if idx >0:
            self.get_logger().info(f'The name {request.name} already exists in Turtlesim Plus. Use {name} instead.')
        if request.x:
            x = request.x
        else:
            x = init_pose[0]
        if request.y:
            y = request.y
        else:
            y = init_pose[1]
        if request.theta:
            theta = request.theta
        else:
            theta = init_pose[2]
        
        image = random.choice(self.simulator.gui.available_images)
        self.simulator.gui.available_images.remove(image)       
        turtle = Turtle(name=name,init_pose=[x,y,theta])
        # load graphics when added
        graphics = pygame.image.load(os.path.join(self.simulator.gui.image_dir,'turtle',image)).convert_alpha()
        turtle.set_graphics(graphics=graphics)
        ros2_turtle = TurtleCommandScannerEatROS2Plugin(turtle=turtle,node=self,image=image)
        self.simulator.add_entity(entity=ros2_turtle)
        response.name = name
        return response
    def remove_turtle_srv_callback(self,request:Kill.Request,response:Kill.Response):
        if not request.name:
            self.get_logger().warning('No name is given. No turtle is removed.')
        else:
            if request.name in self.simulator.entity_list.keys():
                entity = self.simulator.entity_list[request.name]
                if isinstance(entity,TurtleCommandScannerEatROS2Plugin):
                    self.simulator.remove_entity(entity=entity)
                    self.simulator.engine.remove_entity(entity=entity)
                    self.simulator.gui.available_images.append(entity.image)
                    entity.__del__()
                    self.get_logger().warning(f'Successfully remove {request.name}')
            else:
                self.get_logger().warning(f'No turtle with the name {request.name}')
        return response
    def spawn_pizza_srv_callback(self,request:GivePosition.Request,response:GivePosition.Response):
        idx = 0
        name = 'pizza0'+str(idx)
        while name in self.simulator.entity_list.keys():
            idx += 1
            name = 'pizza'+str(idx)
        if not request.x:
            x = random.uniform(0,10.88)
        else:
            x = request.x
        if not request.y:
            y = random.uniform(0,10.88)
        else:
            y = request.y
        # load graphics when added
        graphics = pygame.image.load(os.path.join(self.simulator.gui.image_dir,'object','pizza.png')).convert_alpha()
        pizza = Pizza(name=name,pose=[x,y,0.0])
        pizza.set_graphics(graphics=graphics)
        self.simulator.add_entity(entity=pizza)
        return response
        
    def timer_callback(self):
        self.simulator.step()
