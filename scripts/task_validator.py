#!/usr/bin/env python3
"""Validates an LLM-generated plan against the fixed whitelist of skills,
objects and zones before any step is allowed to execute. This is the
"Plan Validator" stage in the assignment's reference pipeline."""

ALLOWED_SKILLS = {"home", "pick", "place"}


class PlanValidationError(RuntimeError):
    pass


def validate_plan(plan: dict, objects: list, zones: list) -> list:
    """Return the validated list of steps, or raise PlanValidationError."""
    if not isinstance(plan, dict) or not isinstance(plan.get("plan"), list):
        raise PlanValidationError("Plan must be a JSON object with a 'plan' array")

    steps = plan["plan"]
    if not steps:
        raise PlanValidationError("Plan is empty")

    for i, step in enumerate(steps):
        if not isinstance(step, dict) or "skill" not in step:
            raise PlanValidationError(f"Step {i} is missing 'skill'")

        skill = step["skill"]
        if skill not in ALLOWED_SKILLS:
            raise PlanValidationError(
                f"Step {i}: skill '{skill}' is not in the allowed list {sorted(ALLOWED_SKILLS)}"
            )

        if skill in ("pick", "place"):
            obj = step.get("object")
            if obj not in objects:
                raise PlanValidationError(f"Step {i}: object '{obj}' is not a known object {objects}")

        if skill == "place":
            zone = step.get("zone")
            if zone not in zones:
                raise PlanValidationError(f"Step {i}: zone '{zone}' is not a known zone {zones}")

    return steps
