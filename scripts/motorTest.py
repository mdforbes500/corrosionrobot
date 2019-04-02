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
	new_angle = bb_front_angle + 5
	servo_kit.servo[bottom_base_front].angle = new_angle
	bb_front_angle = new_angle

#Call to retract the bottom base front wheels
def retractBBFront():
	new_angle = bb_front_angle - 5
	servo_kit.servo[bottom_base_front].angle = new_angle
	bb_front_angle = new_angle


######################BOTTOM BASE REAR SERVO CODE####################

#Call to extend the bottom base front wheels
def extendBBRear():
	new_angle = bb_rear_angle + 5
	servo_kit.servo[bottom_base_rear].angle = new_angle
	bb_rear_angle = new_angle

#Call to retract the bottom base front wheels
def retractBBRear():
	new_angle = bb_rear_angle - 5
	servo_kit.servo[bottom_base_rear].angle = new_angle
	bb_rear_angle = new_angle



#######################UPPER FRONT WHEEL SERVO EXTEND/RETRACT CODE#########

#call to extend the upper front wheels
def extendUFront():
	new_angle = upper_front_angle + 10
	servo_kit.servo[upper_left_front].angle = new_angle
	servo_kit.servo[upper_right_front].angle = new_angle
	upper_front_angle = new_angle

#call to retrtact the upper front wheels
def retractUFront():
	new_angle = upper_front_angle - 10
	servo_kit.servo[upper_left_front].angle = new_angle
	servo_kit.servo[upper_right_front].angle = new_angle
	upper_front_angle = new_angle



#######################UPPER REAR WHEEL SERVO EXTEND/RETRACT CODE#########

#call to extend the upper rear wheels
def extendURear():
	new_angle = upper_rear_angle + 10
	servo_kit.servo[upper_left_rear].angle = new_angle
	servo_kit.servo[upper_right_rear].angle = new_angle
	upper_rear_angle = new_angle

#call to retrtact the upper rear wheels
def retractURear():
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
	base_kit.motor1.throttle = 0.6
	base_kit.motor2.throttle = 0.6
	base_kit.motor3.throttle = -0.6
	base_kit.motor4.throttle = -0.6

#Call to move the robot backward
def moveBackward():
	base_kit.motor1.throttle = -0.6
	base_kit.motor2.throttle = -0.6
	base_kit.motor3.throttle = 0.6
	base_kit.motor4.throttle = 0.6

#rotates the specified group(front or back) clockWise
def rotateCW(group):
	if group == 0:
		rotate_kit.motor1.throttle = 0.6
		rotate_kit.motor2.throttle = 0.6
	elif group == 1:
		rotate_kit.motor3.throttle = 0.6
		rotate_kit.motor4.throttle = 0.6

#rotates the specified group(front or back) counter-clockWise
def rotateCCW(group):
	if group == 0:
		rotate_kit.motor1.throttle = -0.6
		rotate_kit.motor2.throttle = -0.6
	elif group == 1:
		rotate_kit.motor3.throttle = -0.6
		rotate_kit.motor4.throttle = -0.6

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
			group_val = int(input("Enter what you would like to control...\n\t"\
				"0 for FRONT_ROTATE\n\t 1 for REAR_ROTATE\n\t"\
				"2 for BASE\n"))
			dir_val = int(input("Enter the direction to move...\n\t"\
				"8 for CCW/BACKWARD\n\t, 9 for CW/FORWARD\n"))
			if group_val == 0:
				if dir_val == 8:
					rotateCCW(FRONT)
				elif dir_val == 9:
					rotateCW(FRONT)
				else:
					allStop()
			elif group_val == 1:
				if dir_val == 8:
					rotateCCW(REAR)
				elif dir_val == 9:
					rotateCW(REAR)
				else:
					allStop()
			elif group_val ==2:
				if dir_val == 8:
					moveBackward()
				elif dir_val == 9:
					moveForward()
				else:
					allStop()
			else:
				 allStop()
		elif servo_motor_select == 0:
			group_val = int(input("Enter what you would like to control...\n\t"\
				"0 for bottom base front\n\t 1 for bottom base rear\n\t"\
				"2 for upper front wheels\n\t 3 for upper rear wheels\n\t"\
				"4 for rack servo\n\t 5 for front sensor package servo\n\t"\
				"6 for rear sensor package servo\n"))
			dir_val = int(input("Enter the direction...\n\t"\
				"8 for CCW/RETRACT\n\t, 9 for CW/EXTEND\n"))
			if group_val == 0:
				if dir_val == 8:
					retractBBFront()
				elif dir_val == 9:
					extendBBFront()
				else:
					allStop()
			elif group_val == 1:
				if dir_val == 8:
					retractBBRear()
				elif dir_val == 9:
					extendBBRear()
				else:
					allStop()
			elif group_val ==2:
				if dir_val == 8:
					extendUFront()
				elif dir_val == 9:
					retractUFront()
				elif dir_val == 0:
					allStop()
			elif group_val == 3:
				if dir_val == 8:
					retractURear()
				elif dir_val == 9:
					extendURear()
				else:
					allStop()
			elif group_val ==4:
				if dir_val == 8:
					extendRack()
				elif dir_val == 9:
					retractRack()
				elif dir_val == 0:
					allStop()
			elif group_val == 5:
				if dir_val == 8:
					spinFrontCCW()
				elif dir_val == 9:
					spinFrontCW()
				else:
					allStop()
			elif group_val ==6:
				if dir_val == 8:
					spinRearCCW()
				elif dir_val == 9:
					spinRearCW()
				elif dir_val == 0:
					allStop()



main()
