from adafruit_motorkit import MotorKit
kit = MotorKit()

var = 1
while var == 1:
	num = int(input("Enter a number for speed.\n"\
		"Enter 1 to 9 (slow to fast forwards) -1 to -9 (backwards...)\n"\
		"You can enter 0 to stop and 11 exits the program"))
	print("You entered: ",num)
	value = num * 0.1
	value2 = value * (-1)
	if num == 11:
		var = 0
	else:
		kit.motor1.throttle = value
		kit.motor2.throttle = value
		kit.motor3.throttle = value2
		kit.motor3.throttle = value2

print("Completed Test!")
