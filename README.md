# ur3_llm_control

ROS 2 (Humble) package that controls a UR3 pick-and-place task with natural
language: an LLM (via [9Router](https://github.com/decolua/9router), run
locally) turns a command like *"Put the red cube in zone C."* into a
structured skill plan, which is validated and then executed through
MoveIt 2.

```
Natural Language Command -> LLM Planner -> JSON Plan -> Plan Validator
  -> Skill Executor -> MoveIt 2 -> UR3
```

## Personalization

`student_id = 20020675` → last two digits `75 mod 6 = 3` → **P = 3**:

| Zone | Color |
|---|---|
| A | Yellow |
| B | Blue |
| C | Red |

## Structure

- **`skill_server`** (C++, `src/skill_server.cpp`) — the only node that talks
  to MoveIt. Exposes three services, `/skill/home`, `/skill/pick`,
  `/skill/place`, each moving the UR3 (approach → Cartesian descend →
  Cartesian ascend) and returning `SUCCESS` / `FAILED` / `INVALID_OBJECT` /
  `INVALID_ZONE` / `PLANNING_FAILED`. Tracks which object is currently held
  to reject an out-of-order `place()`. Adds the table as a MoveIt collision
  object on startup so planned paths route around it.
- **`llm_planner.py`** — calls the local LLM (chat-completion endpoint) with
  a system prompt listing the allowed skills/objects/zones, and parses its
  JSON response.
- **`task_validator.py`** — the **Plan Validator**: rejects any step whose
  skill, object or zone isn't in the fixed whitelist before anything is
  allowed to execute.
- **`skill_executor.py`** — the **Skill Executor**: thin ROS 2 service-client
  wrapper around `skill_server`.
- **`task_runner.py`** — entry point. Takes one command, prints the
  `USER COMMAND` / `LLM PLAN` / `EXECUTION` / `TASK SUCCESS` (or `FAILED`)
  terminal report, and runs the validated plan step by step.

## Environment

`worlds/tabletop.sdf`: UR3 + a table + `red_cube` / `yellow_cube` /
`blue_cube` (4 cm) + three zone markers (`zone_a`/`b`/`c`), all at fixed
poses declared in `config/scene.yaml` (no camera/object detection — poses
are known in advance, as the assignment allows). Zone markers are tinted
with this student's P=3 color mapping above.

## Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this-repo-url> ur3_llm_control
vcs import . < ../letter_writer/dependencies.repos   # same UR/MoveIt deps as letter_writer
cd ~/ros2_ws && colcon build --symlink-install
```

## 9Router setup (local LLM gateway)

```bash
npm install -g 9router
9router --skip-update -n -l -H 127.0.0.1     # dashboard at http://127.0.0.1:20128
```

In the dashboard (`Providers`), **OpenCode Free** needs no login and is
ready immediately. Copy your API key from `Endpoint & Key`, then either
export it (preferred — keeps it out of `config/llm_config.yaml`):

```bash
export NINEROUTER_API_KEY=<paste-your-own-key-here>
```

or fill it into `config/llm_config.yaml`'s `llm_api_key` directly.

## Run

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py ur_type:=ur3
```

Then, once Gazebo/MoveIt/`skill_server` are up (~20 s):

```bash
$(ros2 pkg prefix ur3_llm_control)/lib/ur3_llm_control/run_command.sh \
  "Put the red cube in zone C."
```

Example output:

```
USER COMMAND:
Put the red cube in zone C.

LLM PLAN:
- pick(red_cube)
- place(red_cube, zone_c)
- home()

EXECUTION:
pick(red_cube) .............. SUCCESS
place(red_cube, zone_c) ..... SUCCESS
home() ...................... SUCCESS

TASK SUCCESS
```

Other phrasings work the same way, e.g. `"Hãy lấy khối màu vàng và đặt nó
vào ô A."` or `"Move the blue cube to zone B."` — the LLM, not hard-coded
string matching, decides the skills.

> `run_command.sh` invokes `python3` directly with `DYLD_LIBRARY_PATH` set,
> rather than `ros2 run`: on this macOS/RoboStack setup, `ros2 run`'s own
> process indirection was observed to have macOS strip that variable,
> which breaks rclpy's ability to load this package's own service message
> type support at runtime ("type_support is null").

## Notes on the implementation

- **Table collision avoidance**: MoveIt's planning scene has no built-in
  knowledge of anything outside the robot's URDF. Without adding the table
  explicitly (`skill_server`'s `addTableCollisionObject()`), an OMPL joint
  plan from the idle pose to a hover point above the table was collision-free
  from MoveIt's point of view while physically driving the arm into the
  real Gazebo table.
- **No physical grasping**: the stock UR3 has no gripper. `pick`/`place`
  are demonstrated by moving precisely to the object/zone position and
  descending/ascending (the same idea as "pen up / pen down"); which object
  is "held" is tracked purely in `skill_server`'s own state, used to
  validate that `place()` isn't called without a matching `pick()` first.
- **Current-state fetching**: `MoveGroupInterface::getCurrentState()` /
  `getCurrentPose()` require a `/joint_states` message whose `header.stamp`
  is at or after the moment the call starts waiting; on this setup
  `joint_state_broadcaster` was observed to publish `header.stamp = 0`
  while `/clock` (sim time) is already far ahead, so that condition never
  holds and those calls fail every time ("Failed to fetch current robot
  state"). `setStartStateToCurrentState()` doesn't hit this (it resolves
  the current state through a different, working path), so all planning
  calls use that; `computeCartesianPath()`'s two waypoints are passed
  explicitly (the pose the previous step already commanded the arm to)
  rather than fetched.
- **Retry loop**: as in a plain point-to-point or drawing task, MoveIt
  execution on this non-real-time macOS setup can occasionally trip the
  controller's path tolerance under CPU load; every skill step retries
  (`withRetries()`, up to 15 attempts) for the same reason.

## Result

- Video demo: `<Google Drive link — public>`
- GitHub repo: `<this repo, public>`

![Startup](docs/startup.png)
