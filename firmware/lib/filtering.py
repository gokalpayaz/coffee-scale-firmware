"""Simple Kalman filter implementation for single-channel feed"""
from math import fabs


class KalmanFilter:
    """Simple Kalman filter for single values."""

    def __init__(self, measurement_uncertainty, q=0.01, estimation_uncertainty=None) -> None:
        """Initialize the filter.

        The initial estimation uncertainty will be equal to the measurement uncertainty if not provided.

        Args:
            measurement_uncertainty (float): how much do we expect our measurement to vary
            q (float, optional): covariance of the process noise, usually between 0.001 and 1. Defaults to 0.01.
            estimation_uncertainty (Optional[float], optional): will be overwritten when we apply the filter. Defaults
                to None.
        """
        self.err_meas = measurement_uncertainty
        self.err_est = estimation_uncertainty or measurement_uncertainty
        self.q = q
        self.last_estimate = 0.0

    def update_estimate(self, measurement) -> float:
        """Perform filtering on the current measurement.

        Args:
            measurement (float): latest measurement

        Returns:
            float: filtered measurement taking into account previous values and trend
        """
        kalman_gain = self.err_est / (self.err_est + self.err_meas)
        current_estimate = self.last_estimate + kalman_gain * (measurement - self.last_estimate)
        self.err_est = (1.0 - kalman_gain) * self.err_est + fabs(self.last_estimate - current_estimate) * self.q
        self.last_estimate = current_estimate

        return current_estimate


class RobustWeightFilter:
    """Reject isolated spikes, then smooth noise without hiding real pours.

    A short median window removes one-off HX711 outliers. The following EMA
    uses a lower alpha while the scale is steady and a higher alpha after a
    meaningful weight change, balancing a calm display with responsive flow.
    """

    __slots__ = (
        "window_size",
        "stable_alpha",
        "moving_alpha",
        "movement_threshold_g",
        "zero_deadband_g",
        "last_estimate",
        "_samples",
        "_sorted",
        "_sample_count",
        "_next_sample",
        "_initialized",
    )

    def __init__(
        self,
        window_size=5,
        stable_alpha=0.18,
        moving_alpha=0.55,
        movement_threshold_g=0.4,
        zero_deadband_g=0.05,
    ):
        if window_size < 3 or window_size % 2 == 0:
            raise ValueError("window_size must be an odd number of at least 3")
        if not 0.0 < stable_alpha <= moving_alpha <= 1.0:
            raise ValueError("filter alphas must satisfy 0 < stable <= moving <= 1")
        if movement_threshold_g <= 0.0 or zero_deadband_g < 0.0:
            raise ValueError("filter thresholds must be non-negative")

        self.window_size = window_size
        self.stable_alpha = stable_alpha
        self.moving_alpha = moving_alpha
        self.movement_threshold_g = movement_threshold_g
        self.zero_deadband_g = zero_deadband_g
        self._samples = [0.0] * window_size
        self._sorted = [0.0] * window_size
        self.reset()

    def reset(self, value=0.0):
        """Clear prior samples and seed the next estimate."""

        self.last_estimate = float(value)
        self._sample_count = 0
        self._next_sample = 0
        self._initialized = False

    def update_estimate(self, measurement):
        measurement = float(measurement)
        self._samples[self._next_sample] = measurement
        self._next_sample = (self._next_sample + 1) % self.window_size
        if self._sample_count < self.window_size:
            self._sample_count += 1

        median = self._median()
        if not self._initialized:
            estimate = median
            self._initialized = True
        else:
            delta = abs(median - self.last_estimate)
            alpha = (
                self.moving_alpha
                if delta >= self.movement_threshold_g
                else self.stable_alpha
            )
            estimate = self.last_estimate + alpha * (median - self.last_estimate)

        self.last_estimate = estimate
        if abs(estimate) < self.zero_deadband_g:
            return 0.0
        return estimate

    def _median(self):
        # Copy into a preallocated work buffer and insertion-sort only the
        # populated prefix. Five elements keeps this deterministic and cheap.
        for index in range(self._sample_count):
            self._sorted[index] = self._samples[index]

        for index in range(1, self._sample_count):
            value = self._sorted[index]
            position = index - 1
            while position >= 0 and self._sorted[position] > value:
                self._sorted[position + 1] = self._sorted[position]
                position -= 1
            self._sorted[position + 1] = value

        middle = self._sample_count // 2
        if self._sample_count % 2:
            return self._sorted[middle]
        return (self._sorted[middle - 1] + self._sorted[middle]) / 2.0
