import numpy as np

from src.adder_numpy import Adder


class SpikingPID:
    """
    Spiking PID class.
    Note 1: P, I and D term are now not synchronized; P is calculated faster. Need to check the influence of this.
    Note 2: Currently the input values should have 0, otherwise the output will not be correct

    TODO:
    - Make initialize function
    - should include a check or something to check if inputs are ordered correctly
    """

    def __init__(self, Kp, Ki, Kd, input_values, error_values, control_range, dt, with_delay=False):
        self.dt = dt
        self.bound = max(input_values)
        self.input_size = len(input_values)
        self.error_size = len(error_values)
        self.control_size = len(control_range)
        self.values = input_values
        self.error_values = error_values
        self.with_delay = with_delay
        # self.first_step = False
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.control_range = control_range

        self.idx_zero = np.argmin(np.abs(self.error_values))

        # Create the error module
        self.error_sub = Adder((input_values, -input_values), error_values, with_delay=with_delay)

        # Create the integrator
        # the 0.99 term is to make sure that the integral will always perform 1 neuron shift towards the center
        self.int_adder = Adder((error_values, error_values), error_values, with_delay=with_delay)

        # Create the first-order differentiator
        self.error_diff = None
        self.diff_sub = Adder((error_values, -error_values), error_values, with_delay=with_delay)

        # Create the control module
        self.control_adder = Adder(
            (error_values * self.Kp, error_values * dt * self.Ki, error_values * (1 / dt) * self.Kd),
            control_range,
            with_delay=with_delay,
        )
        # self.control_adder = Adder(
        #     (error_values * self.Kp, error_values * dt * self.Ki),
        #     control_range,
        #     with_delay=with_delay,
        # )

        self._init_signals()


    def _init_signals(self):
        '''
        Initialize all signals (close) to zero, especially necessary for the delayed version
        '''
        self.err_signal = np.zeros(self.error_size)
        self.err_signal[self.idx_zero] = 1
        self.int_signal = np.zeros(self.error_size)
        self.int_signal[self.idx_zero] = 1
        self.differential = np.zeros(self.error_size)
        self.differential[self.idx_zero] = 1
        self.control_signal = np.zeros_like(self.control_range)
        self.control_signal[np.argmin(np.abs(self.control_range))] = 1


    def calculate_output(self, measured, target):
        # Calculate the error
        self.err_signal_next = self.error_sub.step(
            (measured, target)
        )
        if self.with_delay:
            self.proportional_spikes = self.err_signal
        else:
            self.proportional_spikes = self.err_signal_next
        self.err_signal = self.err_signal_next

        # Do the integrator step
        self.int_signal_next = self.int_adder.step((self.int_signal, self.err_signal))
        if self.with_delay:
            self.integral_spikes = self.int_signal
        else:
            self.integral_spikes = self.int_signal_next
        self.int_signal = self.int_signal_next

        # Do the differential step
        # initialize signal at (closest to) zero
        if self.error_diff is None:
            self.first_step = True
            self.error_diff = self.err_signal
            # self.error_diff[self.idx_zero] = 1
        self.differential_spikes = self.diff_sub.step((self.error_diff, self.err_signal))
        self.error_diff = self.err_signal  # same

        # Calculate the output control
        self.control_signal_next = self.control_adder.step((self.proportional_spikes, self.integral_spikes, self.differential_spikes))
        # self.control_signal_next = self.control_adder.step((self.proportional_spikes, self.integral_spikes))
        if self.with_delay:
            output = self.control_range[int((self.control_signal == 1).nonzero()[0])]
        else:
            output = self.control_range[int((self.control_signal_next == 1).nonzero()[0])]
        self.control_signal = self.control_signal_next

        return output


class SpikingPIDSplitted:
    """
    Spiking PID class.
    Note 1: P, I and D term are now not synchronized; P is calculated faster. Need to check the influence of this.
    Note 2: Currently the input values should have 0, otherwise the output will not be correct

    TODO:
    - Make initialize function
    - should include a check or something to check if inputs are ordered correctly
    """

    def __init__(self, Kp, 
                        Ki, 
                        Kd, 
                        input_values, 
                        error_values, 
                        error_deriv_values, 
                        control_range, 
                        dt, 
                        with_delay=False, 
                        use_loihi_weights=False):
        self.dt = dt
        self.bound = max(input_values)
        self.input_size = len(input_values)
        self.error_size = len(error_values)
        self.error_diff_size = len(error_deriv_values)
        self.control_size = len(control_range)
        self.values = input_values
        self.error_values = error_values
        self.error_deriv_values = error_deriv_values
        self.with_delay = with_delay
        # self.first_step = False
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.control_range = control_range

        self.idx_zero = np.argmin(np.abs(self.error_values))

        # Create the error module
        self.error_sub = Adder((input_values, -input_values), 
                                error_values, 
                                with_delay=with_delay, 
                                use_loihi_weights=use_loihi_weights)

        # Create the integrator
        # the 0.99 term is to make sure that the integral will always perform 1 neuron shift towards the center
        self.int_adder = Adder((0.9*error_values, error_values), 
                                error_values, 
                                with_delay=with_delay,
                                use_loihi_weights=use_loihi_weights)

        # Create the control module
        self.control_adder = Adder(
            (error_values * self.Kp, error_values * dt * self.Ki, error_deriv_values * self.Kd),
            control_range,
            with_delay=with_delay,
            use_loihi_weights=use_loihi_weights
        )

        self._init_signals()


    def _init_signals(self):
        '''
        Initialize all signals (close) to zero, especially necessary for the delayed version
        '''
        self.err_signal = np.zeros(self.error_size)
        self.err_signal[self.idx_zero] = 1
        self.int_signal = np.zeros(self.error_size)
        self.int_signal[self.idx_zero] = 1
        self.differential = np.zeros(self.error_diff_size)
        self.differential[np.argmin(np.abs(self.error_deriv_values))] = 1
        self.control_signal = np.zeros_like(self.control_range)
        self.control_signal[np.argmin(np.abs(self.control_range))] = 1


    def calculate_output(self, measured, target, measured_deriv_error):
        # Calculate the error
        self.err_signal_next = self.error_sub.step(
            (measured, target)
        )
        if self.with_delay:
            self.proportional_spikes = self.err_signal
        else:
            self.proportional_spikes = self.err_signal_next
        self.err_signal = self.err_signal_next

        # Do the integrator step
        self.int_signal_next = self.int_adder.step((self.int_signal, self.err_signal))
        if self.with_delay:
            self.integral_spikes = self.int_signal
        else:
            self.integral_spikes = self.int_signal_next
        self.int_signal = self.int_signal_next

        # Calculate the output control
        # self.control_signal = self.control_adder.step((self.proportional_spikes, self.integral_spikes, self.differential_spikes))
        self.control_signal_next = self.control_adder.step((self.proportional_spikes, self.integral_spikes, measured_deriv_error))
        if self.with_delay:
            output = self.control_range[int((self.control_signal == 1).nonzero()[0])]
        else:
            output = self.control_range[int((self.control_signal_next == 1).nonzero()[0])]
        self.control_signal = self.control_signal_next

        return output
