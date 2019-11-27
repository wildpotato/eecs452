import car_dir
import motor
import math
from simple_pid import PID

CTRL_DEBUG = False

class Car(object):
    def __init__(self):
        self.origin = (320,425) #(x,y)
        self.on_lane = False
        self.off_lane_cnt = 0
        # Working : 0.33 0.0295 0.12 | 0.33 0.292 0.12 
        self.Kp = 0.33
        self.Ki = 0.0292
        self.Kd = 0.12
        self.pid = PID(self.Kp,self.Ki,self.Kd,setpoint=90)
        # self.pid.output_limits = (45,135)
        self.setup()

    def motor_stop(self):
        motor.ctrl(0)

    def servo_home(self):
        car_dir.home()

    def setup(self):
        car_dir.setup(busnum=1)
        motor.setup(busnum=1)
        self.servo_home()
        self.motor_stop()

    def motor_run(self, speed=35):
        motor.forwardWithSpeed(speed)

    def servo_turn(self, cur_angle=-90):
        if cur_angle == -90:
            return
        if CTRL_DEBUG:
                print("&Before PID angle =", cur_angle)
        steering_angle = int(self.pid(cur_angle)) + 90
        if CTRL_DEBUG:
                print("$After PID angle =", steering_angle)
        car_dir.turn(steering_angle)

    def tune_pid(self):
        self.pid.Kp = self.Kp
        self.pid.Ki = self.Ki
        self.pid.Kd = self.Kd

    def check_stage1_status(self, lanes):
        if lanes is None or len(lanes)==0:
            if self.off_lane_cnt > 5: # no lane any more
                self.on_lane = False
                self.motor_stop()
                self.servo_home()
                return True
            self.off_lane_cnt += 1
            print("No line %d" %self.off_lane_cnt)
        else:
            if len(lanes) == 1:
                self.motor_run(40)
            self.on_lane = True
            self.off_lane_cnt = 0
            return False

    def parking(self, coordinate):
        if coordinate[1] == -1:
            return False
        angle = 90
        # if coordinate[0] > self.origin[0]:
        #     return False
        # x = coordinate[0]-self.origin[0]
        y = self.origin[1] - coordinate[1]
        # # compute distance
        # dist = math.sqrt(x**2 + y**2)
        # # compute angle
        # angle = math.degrees(math.atan2(math.radians(y),math,radians(x))) + 90

        return self.navigate_to_spot(y,angle)

    def navigate_to_spot(self, dist, angle):
        # reached parking spot
        if (dist < 3 and angle > 88 and angle < 92):
            self.motor_stop()
            return True
        # if self.on_lane:
        #     return False

        self.motor_run(25)
        self.servo_turn(angle)
        return False

if __name__ == '__main__':
    car = Car()
