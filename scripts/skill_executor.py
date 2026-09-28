#!/usr/bin/env python3
"""Thin ROS 2 service-client wrapper around skill_server's /skill/home,
/skill/pick and /skill/place services -- the "Skill Executor" stage."""
import rclpy

from ur3_llm_control.srv import Home, Pick, Place


class SkillExecutor:
    def __init__(self, node, timeout_sec: float = 60.0):
        self._node = node
        self._timeout = timeout_sec
        self._home_cli = node.create_client(Home, "skill/home")
        self._pick_cli = node.create_client(Pick, "skill/pick")
        self._place_cli = node.create_client(Place, "skill/place")
        for cli, name in (
            (self._home_cli, "skill/home"),
            (self._pick_cli, "skill/pick"),
            (self._place_cli, "skill/place"),
        ):
            if not cli.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"Service {name} not available -- is skill_server running?")

    def _call(self, client, request) -> str:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        if future.result() is None:
            return "FAILED"
        return future.result().status

    def home(self) -> str:
        return self._call(self._home_cli, Home.Request())

    def pick(self, obj: str) -> str:
        req = Pick.Request()
        req.object = obj
        return self._call(self._pick_cli, req)

    def place(self, obj: str, zone: str) -> str:
        req = Place.Request()
        req.object = obj
        req.zone = zone
        return self._call(self._place_cli, req)
