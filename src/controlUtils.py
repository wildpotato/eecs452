import car_dir
import motor
from simple_pid import PID


def motor_stop():
    motor.ctrl(0)

def servo_home():
    car_dir.home()

def setup():
    car_dir.setup(busnum=1)
    motor.setup(busnum=1)
    servo_forward()
    motor_stop()


def motor_run(speed=35):
    motor.forwardWithSpeed(speed)

def servo_turn(angle):
    if angle == -90:
        return
