"""Dependency-free ContactReport mode resolver for the conveyor robots."""

from __future__ import annotations

import os
from collections.abc import Mapping


CONTACT_REPORT_MODE_ENV = "ISAACLAB_CONVEYOR_CONTACT_REPORT"
DEFAULT_CONTACT_REPORT_MODE = "ankles"
CONTACT_REPORT_MODES = ("ankles", "all", "off")
CONTACT_HISTORY_LENGTH_ENV = "ISAACLAB_CONVEYOR_CONTACT_HISTORY_LENGTH"
DEFAULT_CONTACT_HISTORY_LENGTH = 4
CONTACT_HISTORY_LENGTHS = (0, 4)


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


def resolve_contact_history_length(environ: Mapping[str, str] | None = None) -> int:
    """Return the strict ContactSensor history length, either ``0`` or ``4``.

    ``4`` preserves the historical rolling force buffer.  ``0`` is the
    current-value-only performance candidate and is safe only when the scene
    keeps lazy sensor updates enabled; the scene config enforces that coupled
    runtime invariant.

    Parsing deliberately accepts only the two canonical integer spellings.
    This keeps blank strings, booleans, floats and accidentally unsupported
    intermediate history lengths from silently changing sensor semantics.
    """

    values = os.environ if environ is None else environ
    raw_length = values.get(
        CONTACT_HISTORY_LENGTH_ENV, str(DEFAULT_CONTACT_HISTORY_LENGTH)
    )
    normalized = raw_length.strip()
    if normalized not in {str(length) for length in CONTACT_HISTORY_LENGTHS}:
        choices = ", ".join(str(length) for length in CONTACT_HISTORY_LENGTHS)
        raise ValueError(
            f"{CONTACT_HISTORY_LENGTH_ENV}={raw_length!r} 无效，可选值: {choices}"
        )
    return int(normalized)
