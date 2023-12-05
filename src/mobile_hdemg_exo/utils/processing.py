from scipy import signal


def notch_filter(data, sampling_frequency):
    """ 
    Applies a notch filter to the EMG data.

    Args:
        data: A list of integers representing an EMG reading.
        sampling_frequency: The sampling frequency.

    Returns:
        A list of integers representing an EMG reading with a notch filter applied.
    """
    b, a = signal.iirnotch(60, 30, sampling_frequency)
    return signal.filtfilt(b, a, data)


def butter_bandpass(lowcut, highcut, sampling_frequency, order=2):
    """ 
    Creates a bandpass filter.

    Args:
        lowcut: The lower cutoff frequency.
        highcut: The higher cutoff frequency.
        sampling_frequency: The sampling frequency.
        order: The order of the filter.

    Returns:
        A list of integers representing an EMG reading with a bandpass filter applied.
    """
    nyq = 0.5 * sampling_frequency
    low = lowcut / nyq
    high = highcut / nyq
    b, a = signal.butter(order, [low, high], btype='band')
    return b, a


def csv_output(emg_reading):
    """
    Outputs the EMG reading to a CSV file.

    Args:
        emg_reading: A list of integers representing an EMG reading.
    """
    with open('emg_data.csv', 'a') as f:
        f.write(str(emg_reading) + '\n')
