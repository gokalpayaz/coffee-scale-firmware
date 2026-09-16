"""Pure helpers shared by the interactive HX711 calibration script."""


def calculate_scale_factor(empty_average, loaded_average, known_weight_g):
    """Return raw HX711 counts per gram for a known calibration weight."""

    known_weight_g = float(known_weight_g)
    if known_weight_g <= 0.0:
        raise ValueError("known_weight_g must be positive")

    scale_factor = (float(loaded_average) - float(empty_average)) / known_weight_g
    if scale_factor == 0.0:
        raise ValueError("loaded and empty readings must differ")
    return scale_factor
