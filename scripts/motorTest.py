from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
import time

servo_kit = ServoKit(channels=16)
base_kit = MotorKit(address=0x61) #kit object for first board(CONTROLS BASE MOTORS)
rotate_kit = MotorKit() #kit object for 2nd board(ROTATE MOTORS)

#group assignment
FRONT = 0
REAR = 1
BASE = 2


#speed control signal
speed = 1

#Servo Assignment
bottom_base_front = 0
bottom_base_rear = 1
upper_left_front = 2
upper_right_front = 3
upper_left_rear = 4
upper_right_rear = 5
#continuous servo assignement
rack_servo = 6
front_package = 7
rear_package = 8

#variables used for the angles of the 180 degree servos
bb_front_angle = 90
bb_rear_angle = 90
# ul_front_angle = 90
# ur_front_angle = 90
upper_front_angle = 90
# ul_rear_angle = 90
# ur_rear_angle = 90
upper_rear_angle = 90

#####################RACK SERVO CODE####################################
#call to extend the rack
def extendRack():
	servo_kit.continuous_servo[rack_servo].throttle = 1

#call to retract the rack
def retractRack():
	servo_kit.continuous_servo[rack_servo].throttle = -1

#call to stop the rack from moving
def stopRack():
	servo_kit.continuous_servo[rack_servo].throttle = 0



######################FRONT PACKAGE SERVO CODE#########################

#call to rotate front sensor package CLOCKWISE
def spinFrontCW():
	servo_kit.continuous_servo[front_package].throttle = 1

#call to rotate front sensor package COUNTER-CLOCKWISE
def spinFrontCCW():
	servo_kit.continuous_servo[front_package].throttle = -1

#call to stop the front sensor package from spinning
def stopFront():
	servo_kit.continuous_servo[front_package].throttle = 0



######################REAR PACKAGE SERVO CODE#########################

#call to rotate rear sensor package CLOCKWISE
def spinRearCW():
	servo_kit.continuous_servo[rear_package].throttle = 1

#call to rotate front sensor package COUNTER-CLOCKWISE
def spinRearCCW():
	servo_kit.continuous_servo[rear_package].throttle = -1

#call to stop the front sensor package from spinning
def stopRear():
	servo_kit.continuous_servo[rear_package].throttle = 0



######################BOTTOM BASE FRONT SERVO CODE####################

#Call to extend the bottom base front wheels
def extendBBFront():
	global bb_front_angle
	if bb_front_angle < 180:
		new_angle = bb_front_angle + 5
		servo_kit.servo[bottom_base_front].angle = new_angle
		bb_front_angle = new_angle

#Call to retract the bottom base front wheels
def retractBBFront():
	global bb_front_angle
	if bb_front_angle > 0:
		new_angle = bb_front_angle - 5
		servo_kit.servo[bottom_base_front].angle = new_angle
		bb_front_angle = new_angle


######################BOTTOM BASE REAR SERVO CODE####################

#Call to extend the bottom base front wheels
def extendBBRear():
	global bb_rear_angle
	if bb_rear_angle < 180:
		new_angle = bb_rear_angle + 5
		servo_kit.servo[bottom_base_rear].angle = new_angle
		bb_rear_angle = new_angle

#Call to retract the bottom base front wheels
def retractBBRear():
	global bb_rear_angle
	if bb_rear_angle > 0:
		new_angle = bb_rear_angle - 5
		servo_kit.servo[bottom_base_rear].angle = new_angle
		bb_rear_angle = new_angle



#######################UPPER FRONT WHEEL SERVO EXTEND/RETRACT CODE#########

#call to extend the upper front wheels
def extendUFront():
	global upper_front_angle
	if upper_front_angle < 180:
		new_angle = upper_front_angle + 10
		servo_kit.servo[upper_left_front].angle = new_angle
		servo_kit.servo[upper_right_front].angle = new_angle
		upper_front_angle = new_angle

#call to retrtact the upper front wheels
def retractUFront():
	global upper_front_angle
	if upper_front_angle > 0:
		new_angle = upper_front_angle - 10
		servo_kit.servo[upper_left_front].angle = new_angle
		servo_kit.servo[upper_right_front].angle = new_angle
		upper_front_angle = new_angle



#######################UPPER REAR WHEEL SERVO EXTEND/RETRACT CODE#########

#call to extend the upper rear wheels
def extendURear():
	global upper_rear_angle
	if upper_rear_angle < 180:
		new_angle = upper_rear_angle + 10
		servo_kit.servo[upper_left_rear].angle = new_angle
		servo_kit.servo[upper_right_rear].angle = new_angle
		upper_rear_angle = new_angle

#call to retrtact the upper rear wheels
def retractURear():
	global upper_rear_angle
	if upper_rear_angle > 0:
		new_angle = upper_rear_angle - 10
		servo_kit.servo[upper_left_rear].angle = new_angle
		servo_kit.servo[upper_right_rear].angle = new_angle
		upper_rear_angle = new_angle



#########################################################################
#########################################################################
################ ROBOT MOTOR CONTROL FUNCTIONS ##########################
#########################################################################
#########################################################################

#Call to move the robot forward
def moveForward():
	base_kit.motor1.throttle = -speed
	base_kit.motor2.throttle = -speed
	base_kit.motor3.throttle = speed
	base_kit.motor4.throttle = speed
	rotate_kit.motor1.throttle = -speed
	rotate_kit.motor2.throttle = -speed
	rotate_kit.motor3.throttle = speed
	rotate_kit.motor4.throttle = speed

#Call to move the robot backward
def moveBackward():
	base_kit.motor1.throttle = speed
	base_kit.motor2.throttle = speed
	base_kit.motor3.throttle = -speed
	base_kit.motor4.throttle = -speed
	rotate_kit.motor1.throttle = speed
	rotate_kit.motor2.throttle = speed
	rotate_kit.motor3.throttle = -speed
	rotate_kit.motor4.throttle = -speed


#rotates the specified group(front or back) clockWise
def rotateCW(group):
	if group == 0:
		rotate_kit.motor2.throttle = speed
		rotate_kit.motor4.throttle = speed
		base_kit.motor2.throttle = -speed
		base_kit.motor4.throttle = -speed
	elif group == 1:
		rotate_kit.motor1.throttle = speed
		rotate_kit.motor3.throttle = speed
		base_kit.motor1.throttle = -speed
		base_kit.motor3.throttle = -speed

#rotates the specified group(front or back) counter-clockWise
def rotateCCW(group):
	if group == 0:
		rotate_kit.motor2.throttle = -speed
		rotate_kit.motor4.throttle = -speed
		base_kit.motor2.throttle = speed
		base_kit.motor4.throttle = speed
	elif group == 1:
		rotate_kit.motor1.throttle = -speed
		rotate_kit.motor3.throttle = -speed
		base_kit.motor1.throttle = speed
		base_kit.motor3.throttle = speed
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


#stops ALL MOTORS AND SERVOS
def allStop():
	base_kit.motor1.throttle = 0
	base_kit.motor2.throttle = 0
	base_kit.motor3.throttle = 0
	base_kit.motor4.throttle = 0
	rotate_kit.motor1.throttle = 0
	rotate_kit.motor2.throttle = 0
	rotate_kit.motor3.throttle = 0
	rotate_kit.motor4.throttle = 0
	servo_kit.continuous_servo[rack_servo].throttle = 0
	servo_kit.continuous_servo[front_package].throttle = 0
	servo_kit.continuous_servo[rear_package].throttle = 0




def main():
	var = 1
	while var == 1:
		servo_motor_select = int(input("SERVO CONTROL OR MOTOR CONTROL...\n\t"\
			"0 for SERVO\n\t 1 for MOTOR\n\t"\
			"3 to stop everything\n"))
		if servo_motor_select == 3:
			allStop()
		elif servo_motor_select == 1:
			val = input("Enter what you would like to control...\n\t"\
				"'a' for FRONT_ROTATE CW\n\t 'b' for FRONT_ROTATE CCW\n\t"\
				"'c' for REAR_ROTATE CW\n\t 'd' for REAR_ROTATE CCW\n\t"\
				"'e' for move forward \n\t 'f' for move backward\n\t")
			if val == 'a':
				rotateCW(FRONT)
			elif val == 'b':
				rotateCCW(FRONT)
			elif val == 'c':
				rotateCW(REAR)
			elif val == 'd':
				rotateCCW(REAR)
			elif val == 'e':
				moveForward()
			elif val == 'f':
				moveBackward()
			else:
				allStop()

		elif servo_motor_select == 0:
			val = input("Enter what you would like to control...\n\t"\
				"'a' for BB front extend\n\t 'b' for BB front retract\n\t"\
				"'c' for BB rear extend\n\t 'd' for BB rear retract\n\t"\
				"'e' for UF wheels extend\n\t 'f' for UF wheels retract\n\t"\
				"'g' for UR wheels extend\n\t 'h' for UR wheels retract\n\t"\
				"'i' for rack extend\n\t 'j' for rack retract\n\t"\
				"'k' for front SP CW\n\t 'l' for front SP CCW\n\t"\
				"'m' for rear SP CW\n\t 'n' for rear SP CCW\n\t")
			if val == 'a':
				extendBBFront()
			elif val == 'b':
				retractBBFront()
			elif val == 'c':
				extendBBRear()
			elif val == 'd':
				retractBBRear()
			elif val == 'e':
				extendUFront()
			elif val == 'f':
				retractUFront()
			elif val == 'g':
				extendURear()
			elif val == 'h':
				retractURear()
			elif val == 'i':
				extendRack()
			elif val == 'j':
				retractRack()
			elif val == 'k':
				spinFrontCW()
			elif val == 'l':
				spinFrontCCW()
			elif val == 'm':
				spinRearCW()
			elif val == 'n':
				spinRearCCW()



main()
