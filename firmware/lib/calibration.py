"""HX711 calibration calculation and persistent storage."""


DEFAULT_SCALE_FACTOR = 465.6814

_CALIBRATION_KEY = "cal_factor"
_FIXED_POINT_SCALE = 10000
_MIN_ABS_FACTOR = 1.0
_MAX_ABS_FACTOR = 100000.0


def calculate_scale_factor(empty_average, loaded_average, known_weight_g):
    """Return raw HX711 counts per gram for a known calibration weight."""

    known_weight_g = float(known_weight_g)
    if known_weight_g <= 0.0:
        raise ValueError("known_weight_g must be positive")

    scale_factor = (float(loaded_average) - float(empty_average)) / known_weight_g
    if scale_factor == 0.0:
        raise ValueError("loaded and empty readings must differ")
    return scale_factor


def _validated_factor(value):
    value = float(value)
    absolute = abs(value)
    if value != value or absolute < _MIN_ABS_FACTOR or absolute > _MAX_ABS_FACTOR:
        raise ValueError("scale factor is outside the supported range")
    return value


def _encode_factor(value):
    value = _validated_factor(value)
    return int(round(value * _FIXED_POINT_SCALE))


class CalibrationStore:
    """Persist the load-cell factor as a fixed-point ESP32 NVS integer."""

    __slots__ = ("_nvs", "_default", "_factor", "_has_stored_value")

    def __init__(self, nvs=None, namespace="coffee_scale", default=None):
        if nvs is None:
            import esp32

            nvs = esp32.NVS(namespace)
        if default is None:
            default = DEFAULT_SCALE_FACTOR
        self._nvs = nvs
        self._default = _validated_factor(default)
        self._factor = None
        self._has_stored_value = False

    def load(self):
        """Load a valid factor, falling back safely when NVS is empty/corrupt."""

        if self._factor is not None:
            return self._factor

        try:
            stored = self._nvs.get_i32(_CALIBRATION_KEY)
            factor = _validated_factor(stored / _FIXED_POINT_SCALE)
        except (OSError, ValueError, TypeError, OverflowError):
            factor = self._default
            self._has_stored_value = False
        else:
            self._has_stored_value = True

        self._factor = factor
        return factor

    def save(self, factor):
        """Save a changed factor and return whether NVS was written."""

        encoded = _encode_factor(factor)
        decoded = encoded / _FIXED_POINT_SCALE
        current_encoded = _encode_factor(self.load())
        if self._has_stored_value and encoded == current_encoded:
            return False

        self._nvs.set_i32(_CALIBRATION_KEY, encoded)
        self._nvs.commit()
        self._factor = decoded
        self._has_stored_value = True
        return True
