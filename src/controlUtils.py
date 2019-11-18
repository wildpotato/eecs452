import car_dir
import motor
import math
# from simple_pid import PID
from PID_Controll import PID

class Car(object):
    def __init__(self):
        self.origin = (160,600) #(x,y)
        self.on_lane = False
        self.detected_lanes = 0
        self.Kp = 1
        self.Ki = 0
        self.Kd = 0
        # self.pid = PID(self.Kp,self.Ki,self.Kd,setpoint=90)
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
        # steering_angle = int(self.pid(cur_angle))
        steering_angle = int(self.pid.Compute(cur_angle))
        print("after pid angle =", steering_angle)
        car_dir.turn(steering_angle)

    def tune_pid(self):
        self.pid.Kp = self.Kp
        self.pid.Ki = self.Ki
        self.pid.Kd = self.Kd
        #self.pid.tunings(self.Kp,self.Ki,self.Kd)

    def lane_info(self, lanes):
        if lanes is None:
            self.on_lane = False
            self.detected_lanes = 0
        else:
            self.on_lane = True
            self.detected_lanes = len(lanes)

    def parking(self, coordinate):
        if coordinate[0] > self.origin[0]:
            return False
        x = coordinate[0]-self.origin[0]
        y = coordinate[1]-self.origin[1]
        # compute distance
        dist = math.sqrt(x**2 + y**2)
        # compute angle
        angle = math.degrees(math.atan2(math.radians(y),math,radians(x))) + 90

        return self.navigate_to_spot(dist,angle)

    def navigate_to_spot(self, dist, angle):
        # reached parking spot
        if (dist < 3 and angle > 88 and angle < 92):
            return True
        # if self.on_lane:
        #     return False

        self.motor_run(30)
        self.servo_turn(angle)
        return False

if __name__ == '__main__':
    car = Car()
