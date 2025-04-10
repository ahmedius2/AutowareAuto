import numpy as np
from scipy import stats
from scipy.fft import fft

class SmoothnessMetrics:
    def __init__(self, sampling_rate=10):  # 10 Hz default sampling rate
        self.sampling_rate = sampling_rate

    def calculate_all_metrics(self, time_series):
        """Calculate all smoothness metrics for a given time series."""
        metrics = {
            'rmse': self._calculate_rmse(time_series),
            'spectral_arc_length': self._calculate_spectral_arc_length(time_series),
            'jerk_metric': self._calculate_jerk_metric(time_series),
            'cv': self._calculate_coefficient_variation(time_series)
        }
        
        # Normalize all metrics to 0-100 scale where 100 is smoothest
        normalized_score = self._normalize_metrics(metrics)
        return metrics, normalized_score

    def _calculate_rmse(self, time_series):
        """Calculate Root Mean Square Error from the mean."""
        return np.sqrt(np.mean((time_series - np.mean(time_series))**2))

    def _calculate_spectral_arc_length(self, time_series):
        """Calculate Spectral Arc Length - a measure of movement smoothness."""
        # Compute FFT
        freq_spectrum = fft(time_series)
        frequencies = np.fft.fftfreq(len(time_series), 1/self.sampling_rate)
        
        # Consider only positive frequencies up to Nyquist frequency
        positive_freq_idx = np.where((frequencies > 0) & 
                                   (frequencies <= self.sampling_rate/2))
        
        freq_spectrum = freq_spectrum[positive_freq_idx]
        frequencies = frequencies[positive_freq_idx]
        
        # Normalize spectrum
        freq_spectrum_normalized = freq_spectrum / np.max(np.abs(freq_spectrum))
        
        # Calculate arc length
        diff_spectrum = np.diff(np.abs(freq_spectrum_normalized))
        diff_freq = np.diff(frequencies)
        arc_length = -np.sum(np.sqrt((diff_freq/np.max(frequencies))**2 + 
                                    diff_spectrum**2))
        return arc_length

    def _calculate_jerk_metric(self, time_series):
        """Calculate jerk-based smoothness metric."""
        # Calculate jerk (derivative of acceleration)
        jerk = np.diff(time_series) * self.sampling_rate
        # Integrated squared jerk
        return -np.sum(jerk**2) / len(time_series)

    def _calculate_coefficient_variation(self, time_series):
        """Calculate Coefficient of Variation."""
        return stats.variation(time_series)

    def _normalize_metrics(self, metrics):
        """Combine metrics into a single 0-100 score."""
        # Define weights for each metric
        weights = {
            'rmse': 0.25,
            'spectral_arc_length': 0.35,
            'jerk_metric': 0.25,
            'cv': 0.15
        }
        
        # Normalize and invert metrics so higher is better
        normalized = {}
        for key, value in metrics.items():
            if key in ['rmse', 'cv']:  # Metrics where lower is better
                normalized[key] = 100 * np.exp(-np.abs(value))
            else:  # Metrics where higher (less negative) is better
                normalized[key] = 100 * np.exp(value)
        
        # Calculate weighted average
        final_score = sum(normalized[key] * weights[key] for key in weights)
        return final_score

# Example usage
if __name__ == "__main__":
    # Sample data simulation
    time = np.linspace(0, 10, 100)
    smooth_data = np.sin(time) + np.random.normal(0, 0.1, 100)
    rough_data = np.sin(time) + np.random.normal(0, 0.5, 100)
    
    # Calculate metrics
    metrics_calculator = SmoothnessMetrics(sampling_rate=10)
    
    smooth_metrics, smooth_score = metrics_calculator.calculate_all_metrics(smooth_data)
    rough_metrics, rough_score = metrics_calculator.calculate_all_metrics(rough_data)
    
    print(f"Smooth data score: {smooth_score:.2f}")
    print(f"Rough data score: {rough_score:.2f}")

###The key papers discussing these smoothness metrics in the context of motion and control include:
###
###For Spectral Arc Length:
###
###
###Balasubramanian, S., Melendez-Calderon, A., & Burdet, E. (2012). "A Robust and Sensitive Metric for Quantifying Movement Smoothness" in IEEE Transactions on Biomedical Engineering. This paper introduced the spectral arc length metric.
###
###
###For Jerk-based metrics:
###
###
###Flash, T., & Hogan, N. (1985). "The coordination of arm movements: An experimentally confirmed mathematical model" in Journal of Neuroscience. This is one of the foundational papers introducing jerk-based smoothness metrics.
###
###
###For application in autonomous driving:
###
###
###There's a paper from around 2018 by researchers at Delft University of Technology about comfort metrics in autonomous vehicles, but I'm not entirely confident about the exact citation without being able to verify it.
###
###
###For general movement smoothness metrics:
###
###
###Hogan, N., & Sternad, D. (2009). "Sensitivity of Smoothness Measures to Movement Duration, Amplitude, and Arrests" in Journal of Motor Behavior.
###
###Many of these metrics were originally developed for human motion analysis and robotics, then adapted for autonomous driving. I recommend looking at recent publications from major autonomous driving companies and research institutions for the most current applications of these metrics specifically to autonomous driving.
###
