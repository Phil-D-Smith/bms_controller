
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

    # clear all outputs
    def clear(self):
        # clears PID computations and coefficients
        self.setpoint = 0.0

        self.P_term = 0.0
        self.I_term = 0.0
        self.D_term = 0.0
        self.last_error = 0.0

        # "windup guard" - prevent values (especially I) getting too big
        self.int_error = 0.0
        self.max_gain = 20.0

        self.output = 0.0

    # update outputs
    def update(self, feedback_value):

        # test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        if not (self.setpoint):
            print("no setpoint")
            return -1

        error = self.setpoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.P_term = self.Kp * error
            self.I_term += error * delta_time

            if (self.I_term < -self.max_gain):
                self.I_term = -self.max_gain
            elif (self.I_term > self.max_gain):
                self.I_term = self.max_gain

            self.D_term = 0.0
            if (delta_time > 0):
                self.D_term = delta_error / delta_time

            # remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.P_term + (self.Ki * self.I_term) + (self.Kd * self.D_term)
            return self.output

    # set setpoint and log time it was last set
    def set_setpoint(self, setpoint, max_time=1000000):
        self.setpoint = setpoint
        self.setpoint_time = time.time()
        self.max_time = max_time

    # set gains
    def set_Kp(self, P):
        self.Kp = P

    def set_Ki(self, I):
        self.Ki = I

    def set_Kd(self, D):
        self.Kd = D

    # set max gain
    def set_max(self, max_gain):
        self.max_point = max_gain

    # set sample time
    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    # get last calculated setpoint error
    def get_error(self, last_error):
        self.last_error = last_error
        return last_error
        
    # check error against limits to check value isnt drifting due to hardware failure
    def check_error_limit(self, error_limit):
        # if max settling time has been passed and error is outside limits
        if (time.time() > (self.setpoint_time + self.max_time) and 
            ((self.last_error > error_limit) or (self.last_error < -error_limit))):
            print("setpoint error limit reached")
            return False
        else:
            return True

