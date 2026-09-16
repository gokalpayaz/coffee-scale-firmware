"""Small validated persistence layer for user-visible scale preferences."""

from app_contracts import (
    MODE_ALL,
    MODE_TIMER_FLOW_WEIGHT,
    MODE_TIMER_RATIO_WEIGHT,
    MODE_TIMER_WEIGHT,
    PROFILE_ESPRESSO,
    PROFILE_POUR_OVER,
)


_MODE_KEY = "mode"
_PROFILE_KEY = "profile"
_VALID_MODES = (
    MODE_TIMER_WEIGHT,
    MODE_TIMER_FLOW_WEIGHT,
    MODE_TIMER_RATIO_WEIGHT,
    MODE_ALL,
)
_VALID_PROFILES = (PROFILE_ESPRESSO, PROFILE_POUR_OVER)


class Preferences:
    """Persist only display mode and last auto-profile in ESP32 NVS."""

    __slots__ = ("_nvs", "_mode", "_profile")

    def __init__(self, nvs=None, namespace="coffee_scale"):
        if nvs is None:
            import esp32

            nvs = esp32.NVS(namespace)
        self._nvs = nvs
        self._mode = None
        self._profile = None

    def load(self):
        return self.load_mode(), self.load_profile()

    def load_mode(self):
        if self._mode is None:
            self._mode = self._read_validated(
                _MODE_KEY, _VALID_MODES, MODE_TIMER_WEIGHT
            )
        return self._mode

    def load_profile(self):
        if self._profile is None:
            self._profile = self._read_validated(
                _PROFILE_KEY, _VALID_PROFILES, PROFILE_ESPRESSO
            )
        return self._profile

    def save_mode(self, mode):
        if mode not in _VALID_MODES:
            raise ValueError("invalid display mode")
        if mode == self.load_mode():
            return False
        self._write(_MODE_KEY, mode)
        self._mode = mode
        return True

    def save_profile(self, profile):
        if profile not in _VALID_PROFILES:
            raise ValueError("invalid auto profile")
        if profile == self.load_profile():
            return False
        self._write(_PROFILE_KEY, profile)
        self._profile = profile
        return True

    def _read_validated(self, key, valid_values, default):
        try:
            value = self._nvs.get_i32(key)
        except (OSError, ValueError, TypeError):
            return default
        if value not in valid_values:
            return default
        return value

    def _write(self, key, value):
        self._nvs.set_i32(key, value)
        self._nvs.commit()
