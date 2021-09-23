import numpy as np


def position_coding(input_val, nbins, min_input=-1, max_input=1):
    """
    Linear population coding where the inputs are put in nbins per input feature
    """
    spike_layer = np.zeros(nbins)
    bin_size = (max_input - min_input) / nbins
    neuron_vals = np.linspace(
        max_input - bin_size / 2, min_input + bin_size / 2, nbins
    )
    diff = neuron_vals - input_val
    distance = np.square(diff)
    min_index = np.argmin(distance)
    spike_layer[min_index] = 1
    return spike_layer
