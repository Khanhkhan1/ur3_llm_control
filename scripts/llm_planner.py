#!/usr/bin/env python3
"""Calls the 9Router (OpenAI-compatible) chat completion endpoint and turns
a natural-language command into a structured JSON skill plan.

The LLM is only ever asked to choose and order skills from a fixed
whitelist -- it never sees or produces joint trajectories or robot
commands directly (see task_validator.py for the enforcement step).
"""
import json
import urllib.request

SYSTEM_PROMPT = """Available skills:

pick(object)
place(object, zone)
home()

Objects:
red_cube
yellow_cube
blue_cube

Zones:
zone_a
zone_b
zone_c

You are the task planner for a UR3 pick-and-place robot. Read the user's
natural-language command (English or Vietnamese) and decide which skills to
call, in order, to satisfy it. Always finish the plan with a "home" step
after the requested manipulation is done.

Return ONLY a JSON object of the form:
{"plan": [{"skill": "pick", "object": "red_cube"},
          {"skill": "place", "object": "red_cube", "zone": "zone_b"},
          {"skill": "home"}]}

Do not generate robot joint commands. Do not add any explanation, markdown,
or text outside the JSON object."""


class LLMPlannerError(RuntimeError):
    pass


def get_plan(command: str, base_url: str, api_key: str, model: str, timeout: float = 30.0) -> dict:
    """Call the LLM and return the parsed plan dict: {"plan": [...]}."""
    payload = {
        "model": model,
        "temperature": 0,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": command},
        ],
    }
    req = urllib.request.Request(
        base_url.rstrip("/") + "/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = json.loads(resp.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001 - surface any transport error uniformly
        raise LLMPlannerError(f"LLM request failed: {exc}") from exc

    try:
        content = body["choices"][0]["message"]["content"]
    except (KeyError, IndexError) as exc:
        raise LLMPlannerError(f"Unexpected LLM response: {body}") from exc

    content = content.strip()
    if content.startswith("```"):
        content = content.strip("`")
        if content.lower().startswith("json"):
            content = content[4:]
        content = content.strip()

    try:
        plan = json.loads(content)
    except json.JSONDecodeError as exc:
        raise LLMPlannerError(f"LLM did not return valid JSON: {content!r}") from exc

    if not isinstance(plan, dict) or "plan" not in plan:
        raise LLMPlannerError(f"LLM JSON missing 'plan' key: {plan!r}")

    return plan
