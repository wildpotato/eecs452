import time

class PID(object):

    def __init__(self,
                 Kp=1.0, Ki=0.0, Kd=0.0,
                 setpoint=90)
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint

        self.lastTime = time.time
        self.errSum = 0
        self.lastErr = 0

    def Compute(self, input_val):
        now = time.time
        time_diff = now - self.lastTime
        error = self.setpoint-input_val
        self.errSum = error*time_diff
        dErr = (error - self.lastErr)/time_diff

        Output = self.Kp * error + self.Ki * self.errSum + self.Kd * dErr

        self.lastErr = error
        lastTime = now
