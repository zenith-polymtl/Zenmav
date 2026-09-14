"""
ArduPilot parameter name compatibility (Copter 4.6 and older vs 4.7 and newer).

Copter 4.7 renamed the AC_WPNav parameter group from ``WPNAV_`` to ``WP_`` and
converted its values from centimetres to SI units (m, m/s, m/s/s). Old names do
not exist anymore on 4.7, and new names do not exist on 4.6.

Zenmav accepts either spelling and translates it to the one used by the
connected firmware, converting the value so it stays in the units of the name
you asked for. For example, ``set_param("WPNAV_SPEED", 300)`` (cm/s) writes
``WP_SPD = 3.0`` (m/s) on a 4.7 firmware.

Sources:
- ArduPilot-4.7 libraries/AC_WPNav/AC_WPNav.cpp (convert_parameters, scale 0.01)
- ArduPilot-4.7 ArduCopter/Parameters.cpp (GOBJECTPTR(wp_nav, "WP_", AC_WPNav))
"""

LEGACY = "legacy"  # ArduPilot <= 4.6 names (WPNAV_*, cm units)
SI = "si"  # ArduPilot >= 4.7 names (WP_*, SI units)

# (4.6 name, 4.7 name, factor) with: 4.7 value = 4.6 value * factor
PARAM_RENAMES = [
    ("WPNAV_SPEED", "WP_SPD", 0.01),  # cm/s -> m/s
    ("WPNAV_RADIUS", "WP_RADIUS_M", 0.01),  # cm -> m
    ("WPNAV_SPEED_UP", "WP_SPD_UP", 0.01),  # cm/s -> m/s
    ("WPNAV_SPEED_DN", "WP_SPD_DN", 0.01),  # cm/s -> m/s
    ("WPNAV_ACCEL", "WP_ACC", 0.01),  # cm/s/s -> m/s/s
    ("WPNAV_ACCEL_Z", "WP_ACC_Z", 0.01),  # cm/s/s -> m/s/s
    ("WPNAV_ACCEL_C", "WP_ACC_CNR", 0.01),  # cm/s/s -> m/s/s
    ("WPNAV_JERK", "WP_JERK", 1.0),  # m/s/s/s, prefix change only
    ("WPNAV_TER_MARGIN", "WP_TER_MARGIN", 1.0),  # m, prefix change only
    ("WPNAV_RFND_USE", "WP_RFND_USE", 1.0),  # boolean, prefix change only
]

_LEGACY_TO_SI = {old: (new, factor) for old, new, factor in PARAM_RENAMES}
_SI_TO_LEGACY = {new: (old, 1.0 / factor) for old, new, factor in PARAM_RENAMES}

# Parameter that exists on every Copter firmware, used to detect the naming scheme
PROBE_PARAM = {SI: "WP_RADIUS_M", LEGACY: "WPNAV_RADIUS"}


def naming_for_version(version):
    """Expected naming scheme for an (major, minor, patch) firmware version, or None if unknown."""
    if version is None:
        return None
    return SI if tuple(version[:2]) >= (4, 7) else LEGACY


def translate(param_name: str, naming):
    """Translate a parameter name to the naming scheme of the connected firmware.

    Returns:
        (firmware_name, factor): firmware value = requested value * factor.
        Unknown names, or names already in the right scheme, are returned unchanged with factor 1.0.
    """
    name = param_name.upper()
    if naming == SI and name in _LEGACY_TO_SI:
        return _LEGACY_TO_SI[name]
    if naming == LEGACY and name in _SI_TO_LEGACY:
        return _SI_TO_LEGACY[name]
    return param_name, 1.0
