from adafruit_motorkit import MotorKit
base_kit = MotorKit() #kit object for first board(CONTROLS BASE MOTORS)
rotate_kit = MotorKit(address=0x61) #kit object for 2nd board(ROTATE MOTORS)

#group assignment
FRONT = 0
REAR = 1
BASE = 2


#Call to move the robot forward
def moveForward():
	base_kit.motor1.throttle = 6
	base_kit.motor2.throttle = 6
	base_kit.motor3.throttle = -6
	base_kit.motor4.throttle = -6

#Call to move the robot backward
def moveBackward():
	base_kit.motor1.throttle = -6
	base_kit.motor2.throttle = -6
	base_kit.motor3.throttle = 6
	base_kit.motor4.throttle = 6

#rotates the specified group(front or back) clockWise
def rotateCW(group):
	if group == 0:
		rotate_kit.motor1.throttle = 6
		rotate_kit.motor2.throttle = 6
	elif group == 1:
		rotate_kit.motor3.throttle = 6
		rotate_kit.motor4.throttle = 6

#rotates the specified group(front or back) counter-clockWise
def rotateCCW(group):
	if group == 0:
		rotate_kit.motor1.throttle = -6
		rotate_kit.motor2.throttle = -6
	elif group == 1:
		rotate_kit.motor3.throttle = -6
		rotate_kit.motor4.throttle = -6

#stops the movement of the specified group(front-rotate, rear-rotate, base)
def stop(group):
	if group == 0:
		rotate_kit.motor1.throttle = 0
		rotate_kit.motor2.throttle = 0
	elif group == 1:
		rotate_kit.motor3.throttle = 0
		rotate_kit.motor4.throttle = 0
	elif group == 2:
		base_kit.motor1.throttle = 0
		base_kit.motor2.throttle = 0
		base_kit.motor3.throttle = 0
		base_kit.motor4.throttle = 0
    else:
		allStop()


#stops ALL MOTORS
def allStop():
	base_kit.motor1.throttle = 0
	base_kit.motor2.throttle = 0
	base_kit.motor3.throttle = 0
	base_kit.motor4.throttle = 0
	rotate_kit.motor1.throttle = 0
	rotate_kit.motor2.throttle = 0
	rotate_kit.motor3.throttle = 0
	rotate_kit.motor4.throttle = 0


def main():
	var = 1
	while var == 1:
		group_val = int(input("Enter what you would like to control...\n\t"\
			"0 for FRONT_ROTATE\n\t 1 for REAR_ROTATE\n\t"\
			"2 for BASE\n\t 3 to stop everything\n"))
		dir_val = int(input("Enter the direction to move...\n\t"\
			"8 for CCW/BACKWARD\n\t, 9 for CW/FORWARD\n\t, 0 for STOP\n"))
		if group_val == 0:
			if dir_val == 8:
				rotateCCW(0)
			elif dir_val == 9:
				rotateCW(0)
			else:
				stop(0)
		elif group_val == 1:
			if dir_val == 8:
				rotateCCW(1)
			elif dir_val == 9:
				rotateCW(1)
			else:
				stop(1)
		elif group_val ==2:
			if dir_val == 8:
				moveBackward()
			elif dir_val == 9:
				moveForward()
			elif dir_val == 0:
				stop(2)
		else:
			 allStop()
