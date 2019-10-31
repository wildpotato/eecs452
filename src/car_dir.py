#!/usr/bin/env python
import PCA9685 as servo
import time                # Import necessary modules

def Map(x, in_min, in_max, out_min, out_max):
	if x>in_max:
		x=in_max
	elif x<in_min:
		x=in_min
	mid = (out_min+out_max)/2
	return mid + (out_max-mid) * x / in_max

def setup(busnum=None):
	global leftPWM, rightPWM, homePWM, pwm, max_angle, curPWM, turn_overshot
	leftPWM = 140
	homePWM = 260
	rightPWM = 380
	curPWM = homePWM
	turn_overshot = 10
	max_angle = 44
	offset =0
	try:
		for line in open('config'):
			if line[0:8] == 'offset =':
				offset = int(line[9:-1])
	except:
		print 'config error'
	leftPWM += offset
	homePWM += offset
	rightPWM += offset
	if busnum == None:
		pwm = servo.PWM()                  # Initialize the servo controller.
	else:
		pwm = servo.PWM(bus_number=busnum) # Initialize the servo controller.
	pwm.frequency = 50

# ==========================================================================================
# Control the servo connected to channel 0 of the servo control board, so as to make the
# car turn left.
# ==========================================================================================
def turn_left():
	global leftPWM, curPWM
	pwm.write(0, 0, leftPWM)  # CH0
	curPWM = leftPWM

# ==========================================================================================
# Make the car turn right.
# ==========================================================================================
def turn_right():
	global rightPWM, curPWM
	pwm.write(0, 0, rightPWM)
	curPWM = rightPWM

# ==========================================================================================
# Make the car turn back.
# ==========================================================================================

def turn(angle):
	global curPWM
	angle = Map(angle, -max_angle, max_angle, leftPWM, rightPWM)
	pwm.write(0, 0, angle)
	curPWM = angle

def home():
	global homePWM, curPWM, turn_overshot
	if curPWM>homePWM:
		pwm.write(0, 0, homePWM-turn_overshot) # add overshot
	elif curPWM<homePWM:
		pwm.write(0, 0, homePWM+turn_overshot) # add overshot
	else:
		pwm.write(0, 0, homePWM) 
	curPWM = homePWM
	
def calibrate(x):
	pwm.write(0, 0, homePWM+x)

def test():
	while True:
		turn_left()
		time.sleep(1)
		home()
		time.sleep(1)
		turn_right()
		time.sleep(1)
		home()

if __name__ == '__main__':
	setup()
	home()
