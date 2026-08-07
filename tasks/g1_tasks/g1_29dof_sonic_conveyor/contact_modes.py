"""Dependency-free ContactReport mode resolver for the conveyor robots."""

from __future__ import annotations

import os
from collections.abc import Mapping


CONTACT_REPORT_MODE_ENV = "ISAACLAB_CONVEYOR_CONTACT_REPORT"
DEFAULT_CONTACT_REPORT_MODE = "ankles"
CONTACT_REPORT_MODES = ("ankles", "all", "off")


def resolve_contact_report_mode(environ: Mapping[str, str] | None = None) -> str:
    """Return ``ankles``, ``all`` or ``off`` and reject misspellings.

    ``ankles`` is the task default because the SONIC metrics only consume the
    two ankle-roll sensors.  Production launchers can select ``off`` to remove
    both ContactReport APIs and ContactSensor objects.  ``all`` preserves the
    historical whole-articulation behavior for A/B fallback.
    """

    values = os.environ if environ is None else environ
    raw_mode = values.get(CONTACT_REPORT_MODE_ENV, DEFAULT_CONTACT_REPORT_MODE)
    mode = raw_mode.strip().lower() or DEFAULT_CONTACT_REPORT_MODE
    if mode not in CONTACT_REPORT_MODES:
        choices = ", ".join(CONTACT_REPORT_MODES)
        raise ValueError(
            f"{CONTACT_REPORT_MODE_ENV}={raw_mode!r} 无效，可选值: {choices}"
        )
    return mode
