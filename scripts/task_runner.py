#!/usr/bin/env python3
"""Entry point: takes one natural-language command, asks the LLM for a
structured plan, validates it, then executes it step by step through
skill_server (MoveIt 2 -> UR3/UR3e).

    User Command -> LLM -> JSON Plan -> Plan Validator -> Skill Executor
    -> MoveIt 2 -> UR3/UR3e

Usage:
    ros2 run ur3_llm_control task_runner.py --ros-args -p command:="Put the red cube in zone B."
"""
import os
import sys

import rclpy
from rclpy.node import Node

from llm_planner import LLMPlannerError, get_plan
from task_validator import PlanValidationError, validate_plan
from skill_executor import SkillExecutor


def format_call(step: dict) -> str:
    skill = step["skill"]
    if skill == "pick":
        return f"pick({step['object']})"
    if skill == "place":
        return f"place({step['object']}, {step['zone']})"
    return "home()"


def format_execution_line(call: str, status: str, width: int = 28) -> str:
    dots = "." * max(1, width - len(call))
    return f"{call} {dots} {status}"


class TaskRunnerNode(Node):
    def __init__(self):
        super().__init__("task_runner")
        self.declare_parameter("command", "")
        self.declare_parameter("llm_base_url", "http://127.0.0.1:20128/v1")
        self.declare_parameter("llm_model", "oc/muse-spark-1.3-contributor-free")
        self.declare_parameter("llm_api_key", "")
        self.declare_parameter("objects.names", [""])
        self.declare_parameter("zones.names", [""])


def main():
    rclpy.init()
    node = TaskRunnerNode()

    command = node.get_parameter("command").value
    if not command and len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
        command = sys.argv[1]
    if not command:
        print("ERROR: no command given. Pass -p command:=\"...\" or as the first argument.")
        rclpy.shutdown()
        sys.exit(1)

    base_url = node.get_parameter("llm_base_url").value
    model = node.get_parameter("llm_model").value
    # Prefer an environment variable over the parameter file so the real
    # key never has to be committed to config/llm_config.yaml.
    api_key = os.environ.get("NINEROUTER_API_KEY") or node.get_parameter("llm_api_key").value
    objects = list(node.get_parameter("objects.names").value)
    zones = list(node.get_parameter("zones.names").value)

    print("USER COMMAND:")
    print(command)
    print()

    try:
        raw_plan = get_plan(command, base_url, api_key, model)
    except LLMPlannerError as exc:
        print(f"LLM PLANNING FAILED: {exc}")
        rclpy.shutdown()
        sys.exit(1)

    try:
        steps = validate_plan(raw_plan, objects, zones)
    except PlanValidationError as exc:
        print("LLM PLAN (rejected):")
        print(raw_plan)
        print()
        print(f"PLAN REJECTED: {exc}")
        print()
        print("TASK FAILED")
        rclpy.shutdown()
        sys.exit(1)

    print("LLM PLAN:")
    for step in steps:
        print(f"- {format_call(step)}")
    print()

    executor = SkillExecutor(node)

    print("EXECUTION:")
    task_ok = True
    for step in steps:
        call = format_call(step)
        skill = step["skill"]
        if skill == "home":
            status = executor.home()
        elif skill == "pick":
            status = executor.pick(step["object"])
        else:
            status = executor.place(step["object"], step["zone"])

        print(format_execution_line(call, status))
        if status != "SUCCESS":
            task_ok = False
            break

    print()
    print("TASK SUCCESS" if task_ok else "TASK FAILED")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if task_ok else 1)


if __name__ == "__main__":
    main()
