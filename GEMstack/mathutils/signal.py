import scipy.signal as signal
from typing import Sequence

class OnlineLowPassFilter(object):
    def __init__(self, cutoff : float, sampling_frequency : float, order : int):
        
        nyq = 0.5 * sampling_frequency
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='lowpass', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def __call__(self, data: float):
        filtered, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filtered[0]

    def reset(self) -> None:
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def filter(self, time_series : Sequence[float]) -> Sequence[float]:
        """Filters a time series"""
        return signal.lfilter(self.b, self.a, time_series)
