import numpy as np
import rospy

def to_loihi(weights, thresholds):
    abs_max = np.max(np.abs(weights))
    weights_loihi = ((weights + abs_max) / (2 * abs_max)) * 254
    # print(weights_loihi - 127)
    # print(2 * np.sign(weights_loihi - 127) * np.ceil(np.abs(weights_loihi - 127)))
    weights_loihi = (2 * np.sign(weights_loihi - 127) * np.ceil(np.abs(weights_loihi - 127))).astype('i')
    thresholds_loihi = ((thresholds + abs_max) / (2 * abs_max)) * 254
    thresholds_loihi = np.floor(thresholds_loihi).astype('i') * 2 - 254
    return weights_loihi, thresholds_loihi

def create_csv_from_adder(name, adder):
    if not adder.use_loihi_weights:
        first_weights_loihi, first_thresholds_loihi = to_loihi(adder.first_layer_weights, adder.first_layer_thresholds)
        second_weights_loihi, second_thresholds_loihi = to_loihi(adder.second_layer_weights, adder.second_layer_thresholds)
    else:
        first_weights_loihi, first_thresholds_loihi = adder.first_layer_weights, adder.first_layer_thresholds
        second_weights_loihi, second_thresholds_loihi = adder.second_layer_weights, adder.second_layer_thresholds

    np.savetxt(f"{name}_first_weights.csv", first_weights_loihi, fmt='%i', delimiter=",")
    np.savetxt(f"{name}_first_thresholds.csv", first_thresholds_loihi, fmt='%i', delimiter=",")
    np.savetxt(f"{name}_second_weights.csv", second_weights_loihi, fmt='%i', delimiter=",")
    np.savetxt(f"{name}_second_thresholds.csv", second_thresholds_loihi, fmt='%i', delimiter=",")

class Adder:
    """
    Class that is a spiking adder, where both the previous and
    current values are place-coded spiking neurons
    TODO:
    - Make initialize function for states
    - Keep track of the states internally, no need outside.
    """

    def __init__(self, inputs, output_range, with_delay=False, use_loihi_weights=False):
        super(Adder, self).__init__()

        self.with_delay = with_delay
        self.input_size = sum([len(x) for x in inputs])
        self.hidden_size = len(output_range)  # has to be same as input for integrator
        self.output_range = output_range
        self.hidden = None
        self.use_loihi_weights = use_loihi_weights
        # below numerical precision can cause issues because both positive and negative should contain zero.
        # fixed but is not really clean right now.

        self.output_pos = self.output_range[
            (self.output_range >= -0.0001).nonzero()
        ]
        self.output_neg = self.output_range[
            (self.output_range <= 0.0001).nonzero()
        ]
        self.pos_len = len(self.output_pos)
        self.neg_len = len(self.output_neg)

        pos_weights = np.concatenate(inputs, 0)
        
        self.neg_weights = np.broadcast_to(-pos_weights, [self.neg_len, self.input_size])
        self.pos_weights = np.broadcast_to(pos_weights, [self.pos_len, self.input_size])
        self.pos_thresholds = self.output_pos
        self.neg_thresholds = np.abs(self.output_neg)

        weights = np.concatenate((self.pos_weights, self.neg_weights), 0)
        thresholds = np.concatenate((self.pos_thresholds, self.neg_thresholds), 0)
        if use_loihi_weights:
            self.first_layer_weights, self.first_layer_thresholds = to_loihi(weights, thresholds)
        else:
            self.first_layer_weights, self.first_layer_thresholds = weights, thresholds

        pos_err_weights = np.diag(self.pos_thresholds) - np.diag(
            np.ones(self.pos_len - 1), k=1
        )

        neg_err_weights = np.diag(self.neg_thresholds) - np.diag(
            np.ones(self.neg_len - 1), k=-1
        )
        second_layer_weights = np.zeros(
            [self.pos_len + self.neg_len, len(self.output_range)]
        )
        second_layer_weights[: self.pos_len, : self.pos_len] = pos_err_weights
        second_layer_weights[-self.neg_len :, -self.neg_len :] = neg_err_weights
        
        if use_loihi_weights:
            self.second_layer_weights, self.second_layer_thresholds = to_loihi(second_layer_weights.T, np.abs(self.output_range))
        else:
            self.second_layer_weights, self.second_layer_thresholds = second_layer_weights.T, np.abs(self.output_range)

    def step(self, inputs):
        input_values = np.concatenate(inputs, 0)

        # self.hidden_potential = self.first_layer_weights @ input_values
        self.hidden_next = (self.first_layer_weights @ input_values) >= self.first_layer_thresholds
        if self.with_delay:
            if self.hidden is None:
                self.hidden = np.zeros_like(self.hidden_next)
                self.hidden[np.argmin(np.abs(self.first_layer_thresholds))] = 1
                self.output = np.zeros_like(self.second_layer_weights @ self.hidden)
                self.output[np.argmin(np.abs(self.output_range))] = 1
            else:
                self.output = (self.second_layer_weights @ self.hidden) >= self.second_layer_thresholds
        else:
            self.hidden = self.hidden_next
            self.output = (self.second_layer_weights @ self.hidden) >= self.second_layer_thresholds

        self.hidden = self.hidden_next
        return self.output