


import time


class PID_controller:

    def __init__(self, P=0.2, I=0.0, D=0.0):
        # default coefficients
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        # get time for this loop
        self.current_time = time.time()
        # save last time this loop ran
        self.last_time = self.current_time

        # clears
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.setpoint = 0.0

        self.P_term = 0.0
        self.I_term = 0.0
        self.D_term = 0.0
        self.last_error = 0.0

        # "windup guard" - prevent values (especially I) getting too big
        self.int_error = 0.0
        self.max = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.setpoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.P_term = self.Kp * error
            self.I_term += error * delta_time

            if (self.I_term < -self.max):
                self.I_term = -self.max
            elif (self.I_term > self.max):
                self.I_term = self.max

            self.D_term = 0.0
            if delta_time > 0:
                self.D_term = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.P_term + (self.Ki * self.I_term) + (self.Kd * self.D_term)

    def setKp(self, proportional_gain):
        """Set points"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setMax(self, max):
    
        self.max_point = max

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time
        
    def getError(self, last_error):
        
        self.last_error = last_error
        
        
    