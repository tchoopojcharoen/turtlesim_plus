# turtlesim_plus

A ROS 2 (Humble) reimplementation of `turtlesim` in `pygame`, extended with **interactive
food and delivery mechanics** on top of stock turtlesim's teleop/pen/teleport model: turtles
can scan for nearby entities with a cone-shaped sensor, eat spawned pizzas, and carry parcels
to a drop-off zone. Multiple turtles are supported natively — each is a self-contained
composition of ROS2 plugins (command/scanner/eat/delivery), not a hardcoded singleton.

![The turtlesim_plus pygame window: a turtle spins in place scanning, then drives toward and eats a nearby pizza slice, then resumes scanning](docs/forage_demo.gif)

*The `turtlesim_plus_node` window driven autonomously by
[`turtlesim_plus_bt`](../turtlesim_plus_bt)'s forage behavior tree — no manual teleop.
Captured by running the live simulator + `forage_bt_node`, spawning pizzas via
`/spawn_pizza`, and screenshotting the `Turtlesim+` pygame window roughly every 0.15s (each
capture itself takes ~30ms) with ImageMagick's `import -window`, assembled into a GIF with
`convert -delay 15 -layers Optimize`. Pen
trail turned off for this capture (`/[name]/set_pen ... off: 1`) to keep the frame readable.*

## Nodes / Scripts

- **`turtlesim_plus_node.py`** — the simulator itself. Owns the `pygame` window and every
  ROS2 service/topic/action described below; spawns `turtle1` automatically on startup.
- **`pizza_on_click.py`** — subscribes to `/mouse_position` (published by the simulator on
  every left-click in the pygame window) and calls `/spawn_pizza` at that world position.
  Purely a convenience client — it owns no simulation state of its own.

## Architecture

```text
Simulator (turtlesim_plus/entity.py)
├── Engine            -- steps every PhysicsEntity each tick (fixed dt = time_step param)
├── GUI                -- owns the pygame window; renders GraphicsEntity subclasses in an
│                          explicit, caller-registered order (register_render_order) so new
│                          entity types don't require editing GUI itself
└── entity_list        -- every spawned Turtle/Pizza/Parcel/DropZone, by name

TurtlePlugin (ros2_plugins.py)  -- one per spawned turtle
├── owns a single physics Turtle (entity.py) -- exact matrix-exponential kinematics
├── TurtleCommandROS2Plugin   -- cmd_vel sub, pose pub, stop/set_pen/teleport_* services
├── TurtleScannerROS2Plugin   -- wide-cone scan -> /scan topic + detect_pizza action
├── TurtleEatROS2Plugin       -- narrow-cone eat range -> /eat service, /pizza_count
└── TurtleDeliveryROS2Plugin  -- narrow-cone pickup range -> /pickup, /dropoff, /parcel_count
```

`TurtlePlugin` **composes** the four ROS2 plugins above as separate owned objects rather
than inheriting from all of them — an earlier version (`TurtleCommandScannerEatROS2Plugin`)
triple-inherited them, which forced manually-ordered chained `__init__`/`update`/`render`
calls and caused the turtle sprite to be redrawn three times per frame. Composition fixed
both; see the class docstring in `ros2_plugins.py` for the detail. Every world↔screen pixel
conversion goes through a single module, `world.py` (`WORLD_SIZE = 10.88`,
`SCREEN_SIZE = 500`, matching stock turtlesim's dimensions), rather than being duplicated
per call site.

## Prerequisites

- Ubuntu 22.04, ROS 2 Humble.
- `pygame`, `numpy` (pip; see `requirements.txt`/`rosdep.yaml`).
- Stock `turtlesim` — reused directly for `Pose`, `Spawn`, `Kill`, `SetPen`,
  `TeleportAbsolute`, `TeleportRelative` message/service types rather than redefining them.
- The sibling package **[`turtlesim_plus_interfaces`](../turtlesim_plus_interfaces)** — the
  custom `GivePosition` service, `ScannerData`/`ScannerDataArray` messages, and `GetData`
  action used above. Must be built alongside this package.

## Build

```bash
source /opt/ros/humble/setup.bash
# from the workspace root, one level above src/
source dependencies_install.bash
colcon build --packages-select turtlesim_plus turtlesim_plus_interfaces
source install/setup.bash
```

## Launching

```bash
ros2 launch turtlesim_plus turtlesim_plus.launch.py
```

Starts `turtlesim_plus_node` and `pizza_on_click.py` together (replacing the two-terminal
`ros2 run` workflow), and exposes five of the seven node parameters below as launch
arguments: `time_step`, `scanner_radius`, `scanner_angle_range`, `eat_radius`,
`eat_angle_range` (e.g. `ros2 launch turtlesim_plus turtlesim_plus.launch.py
scanner_radius:=6.0`). `pickup_radius`/`pickup_angle_range` aren't wired into the launch
file yet — pass them directly if you need non-default delivery tuning:

```bash
ros2 run turtlesim_plus turtlesim_plus_node.py --ros-args -p pickup_radius:=3.0
ros2 run turtlesim_plus pizza_on_click.py   # separate terminal
```

## Configuration

All are `TurtlesimPlusNode` ROS2 parameters (`declare_parameter`, read once at startup —
apply to every turtle spawned afterward, not per-turtle):

| Parameter | Default | Meaning |
|---|---|---|
| `time_step` | `0.01` | Physics step (s); also the nominal simulator tick period |
| `scanner_radius` | `4.0` | General `/scan` sensor radius (world units) |
| `scanner_angle_range` | `π/3` (60°) | General `/scan` sensor cone, centered on heading |
| `eat_radius` | `2.0` | `/eat` detection radius |
| `eat_angle_range` | `π/3` (60°) | `/eat` detection cone |
| `pickup_radius` | `2.0` | `/pickup` detection radius |
| `pickup_angle_range` | `π/3` (60°) | `/pickup` detection cone |

## Services / Topics Reference

**Global** (not turtle-specific):

| Name | Type | Notes |
|---|---|---|
| `/spawn_turtle` | `turtlesim/srv/Spawn` | `x`/`y`/`theta` of `NaN` (or omitted) → centered at `(WORLD_SIZE/2, WORLD_SIZE/2, 0)`. A name collision appends `_1`, `_2`, ... rather than discarding the requested name |
| `/remove_turtle` | `turtlesim/srv/Kill` | By name |
| `/spawn_pizza` / `/spawn_parcel` | `turtlesim_plus_interfaces/srv/GivePosition` | `x`/`y` of `NaN` → uniform-random position in `[0, WORLD_SIZE]` |
| `/clear` | `std_srvs/srv/Empty` | Erases every turtle's pen trail (matches stock turtlesim's `/clear`) |
| `/mouse_position` | `geometry_msgs/msg/Point` (pub) | World-coordinate of the last left-click in the pygame window; consumed by `pizza_on_click.py` |

**Per turtle** (`/[name]/...`):

| Name | Type | Direction | Notes |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/Twist` | sub | `linear.x` forward speed, `angular.z` turn rate |
| `pose` | `turtlesim/msg/Pose` | pub | `linear_velocity`/`angular_velocity` fields are always `0.0` (unused) |
| `scan` | `turtlesim_plus_interfaces/msg/ScannerDataArray` | pub | **Only published when at least one entity is in the cone** — no message means nothing detected, not an empty-list message |
| `pizza_count` / `parcel_count` | `std_msgs/msg/Int64` | pub | Published every tick regardless of change |
| `carrying_parcel` | `std_msgs/msg/Bool` | pub | Whether a pickup is currently held |
| `eat` / `stop` / `pickup` / `dropoff` | `std_srvs/srv/Empty` | service | No-op (not an error) if nothing is in range; `dropoff` also requires being within the (invisible) drop-off zone near the world's far corner |
| `set_pen` | `turtlesim/srv/SetPen` | service | Pen is **on by default** (white, width 3), matching stock turtlesim. ⚠️ `ros2 service call` CLI note: YAML parses a bare `off:` key as the boolean `False`, not the string `"off"` — quote it (`'off': 1`) or the call fails with a cryptic `getattr()` error. Real clients (rclpy/rclcpp) are unaffected |
| `teleport_absolute` / `teleport_relative` | `turtlesim/srv/TeleportAbsolute` / `TeleportRelative` | service | Breaks the pen trail into a new stroke rather than drawing a line across the jump |
| `detect_pizza` | `turtlesim_plus_interfaces/action/GetData` | action | One-shot: succeeds with the current scan's Pizza entries if any, aborts otherwise |

## How It Works

1. **Entity model** (`entity.py`) — `Entity` → `GraphicsEntity`/`PhysicsEntity` (both, for
   things that move and render, like `Turtle`). `Pizza`/`Parcel`/`DropZone` are
   `GraphicsEntity`-only (static, no physics step).
2. **Kinematics** — `Turtle.update()` integrates `cmd_vel` via an exact
   matrix-exponential/rotation-matrix formula (not simple Euler), correctly handling the
   curved-arc case (`angular.z != 0`) with a `dt/dtheta`-normalized integral so it converges
   smoothly to straight-line motion as `angular.z → 0` rather than as a separate special
   case.
3. **Sensing** — `Scanner.measure()` filters candidates by both distance and a heading-relative
   angular cone, and by registered detection type (`add_detection_type`) — e.g. the eat-range
   scanner only ever considers `Pizza`, never `Parcel`/`DropZone`/other turtles. Distances
   below a small epsilon bypass the angle check entirely, since bearing is numerically
   unstable (effectively random sign/magnitude) for a target that's ~coincident with the
   turtle.
4. **Pen trail** — `Turtle.trail` is a list of "strokes" (point lists); teleporting or
   toggling the pen back on starts a new stroke so the drawn line never bridges a jump or a
   pen-up gap. A point is only appended when position actually changes tick-to-tick, so a
   stationary turtle doesn't accumulate a solid blob at one spot.
5. **Coordinates** — `world.py` is the single source of truth for the world↔screen mapping
   (`WORLD_SIZE=10.88` world units ↔ `SCREEN_SIZE=500` px), used by every render/mouse-click
   call site.

## Known limitations / Troubleshooting

- **No automated test suite.** Every change this session was verified manually: build,
  launch, exercise the relevant services/topics, and visually confirm via the pygame window
  (screenshotted with ImageMagick where a written record was useful).
- **No inter-entity collision.** `Scanner` only measures distance/angle; nothing prevents
  turtles or objects from overlapping.
- **Pen trail has no size cap.** Long sessions with a lot of movement accumulate large
  per-turtle point lists (no trimming/decimation) — call `/clear` periodically if that
  matters for your use case.
- **Tick rate degrades under many accumulated entities.** `Scanner.measure()` is
  `O(entities)` per scanner, and each turtle runs three of them (scan/eat/pickup) every
  tick. A long session with many spawned-but-never-eaten pizzas (e.g. from repeated
  `pizza_on_click` clicks) can push the simulator well below its nominal `1/time_step` tick
  rate — there's currently no service to bulk-remove uneaten entities, only `/clear` for pen
  trails.
- **`ros2 service call ... set_pen` CLI quirk** — see the Services table above.

## Companion package: behavior-tree control

**[`turtlesim_plus_bt`](../turtlesim_plus_bt)** is a `py_trees`/`py_trees_ros` behavior tree
that drives a turtle autonomously against this package's public ROS2 interface only (no
internal imports) — the "forage" tree shown in the GIF above: scan, chase, and eat pizzas,
spinning in place and then relocating to a random point after a full revolution without
detecting anything. See its own README/source for the tree structure.

## Testing

No automated tests exist yet. Manual verification loop used throughout development:

```bash
colcon build --packages-select turtlesim_plus turtlesim_plus_interfaces
source install/setup.bash
ros2 launch turtlesim_plus turtlesim_plus.launch.py
# in other terminals: ros2 service call / ros2 topic echo / ros2 topic pub against
# the services and topics listed above, and watch the pygame window
```

## License

GPL-3.0-only — see [`LICENSE`](LICENSE).

## Author

Pi Thanacha Choopojcharoen (thanachachoo@gmail.com)
