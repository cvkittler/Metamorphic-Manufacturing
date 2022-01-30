#!/usr/bin/env python3
####################################################
## This code runs the End of Arm Tooling for the 
##  WPI Metamorphic Manufacturing Robot, 2021-2022
##  and is controlled from an external ROS Melodic
##  node
####################################################
## {Licesense}
####################################################
## Authors: Sean Barry, Charles Kittler,
##			Jonathan Landay, Jacob Mackenzi,
##			Aidan Melgar, Patrick Siegler
## Copyright: 2022, MMMQP
## Credits:
## License: {}
## Version: 0.1.5
## Maintainer: {}
## Email: gr-metamorphicmanufacturingmqp2021@wpi.edu
## Status: "Dev"
#################################################### 

# Imports
#import rospy
import time
import threading

# Libs
import RPi.GPIO as GPIO

#Autorun Setup (If True, will not prompt confirmation)
autorun = False

# Raspberry Pi Pin Declarations

# Stepper Motor Control Pins
EN_L = 3 #{IO Pin}
EN_R = 5 #{IO Pin}
DIR_L = 7 #{IO Pin}
DIR_R = 8 #{IO Pin}
STEP_L = 12 #{PWM Pin}
STEP_R = 33 #{PWM Pin}

# Limit Switch Pins (False == not pressed)
Sen_L_max = 10 #{IO Pin} for too close to center
Sen_L_min = 11 #{IO Pin} for too open
Sen_R_max = 13 #{IO Pin} for too close to center
Sen_R_min = 15 #{IO Pin} for too open

e_stop_pin = 16 #{IO Pin} True = e_stop
e_stop_out = 18 #{IO Pin} True = e_stop

# Variables
distance_per_rotation = 5 #millimeters
steps_per_rotation = 6400
distance_per_step = distance_per_rotation/steps_per_rotation #0.00078125 mm/step
steps_per_mm = steps_per_rotation/distance_per_rotation
duty_cycle = 50


left_pos = 0; #distance from max open in mm
right_pos = 0; #distance from max open in mm

e_stop_triggered = False
zeroing = False


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(STEP_L,GPIO.OUT)
GPIO.setup(STEP_R,GPIO.OUT)

ls = GPIO.PWM(STEP_L,500)
rs = GPIO.PWM(STEP_R,500)

## Interrupts 
    
def trigger_e_stop(channel):
	#Stops the manipulator when external E stop initiated
	print('Emergency Stop Initated from IRC5')
	e_stop(channel)

def trigger_L_max(channel):
	#Stops the manipulator when left manipulator exceeds intended operating bounds
	print('Left Manipulator exceeded maximum bounds (too closed)')
	e_stop(channel)
	
def trigger_L_min(channel):
	#Stops the manipulator when left manipulator exceeds intended operating bounds (when not performing a zeroing operation)
	
	if not zeroing:
		print('Left Manipulator exceeded minimum bounds (too open)')
		e_stop(channel)
	
	
def trigger_R_max(channel):
	#Stops the manipulator when right manipulator exceeds intended operating bounds
	print('Right Manipulator exceeded maximum bounds (too closed)')
	e_stop(channel)
	

def trigger_R_min(channel):
	#Stops the manipulator when right manipulator exceeds intended operating bounds (when not performing a zeroing operation)
	if not zeroing:
		print('Right Manipulator exceeded minimum bounds (too open)')
		e_stop(channel)
	

###############################################################################
## Functions
	
def e_stop(channel):
	#disables both motors, kills the PWM
	print('Emergency Stop')
	global ls
	global rs
	GPIO.output(EN_L,GPIO.LOW)
	GPIO.output(EN_R,GPIO.LOW)
	ls.stop()
	rs.stop()
	
	e_stop_triggered = True
	
	#TODO: Return E Stop to IRC5
	
def setup_info():
	#prints pin info and startup information
	print('Setting Up WPI MMMQP End Effector\n\n')
	print('Ensure Pins are wired as follows:\n')
	
	print('Safety Pins:\n')
	print('E Stop FROM IRC5 on pin: ' + str(e_stop_pin))
	print('E Stop TO IRC5 on pin: ' + str(e_stop_out))
	print('Left Manipulator Max (Closest to center): ' + str(Sen_L_max)
          )
	print('Left Manipulator Min (Closest to outside): ' + str(Sen_L_min))
	print('Right Manipulator Max (Closest to center): ' + str(Sen_R_max))
	print('Right Manipulator Min (Closest to outside): ' + str(Sen_R_min))
	
	print('\n Motor Control Pins:\n')
	print('Left Enable: ' + str(EN_L))
	print('Left Direction: ' + str(DIR_L))
	print('Left Pulse/Step: ' + str(STEP_L))
	print('Right Enable: ' + str(EN_R))
	print('Right Direction: ' + str(DIR_R))
	print('Right Pulse/Step: ' + str(STEP_R))
	
	print('\n\n\n')    

def setup():
	#Initializes pins, IRC5 EStop and PWM
	
	print('RaspberryPi Pin Configuration in progress....')
	
	GPIO.setup(EN_L,GPIO.OUT)
	GPIO.setup(EN_R,GPIO.OUT)
	GPIO.setup(DIR_L,GPIO.OUT)
	GPIO.setup(DIR_R,GPIO.OUT)
	GPIO.setup(Sen_L_max, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(Sen_L_min, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(Sen_R_max, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(Sen_R_min, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(e_stop_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	
	GPIO.setup(e_stop_out, GPIO.OUT)
	
	GPIO.add_event_detect(e_stop_pin, GPIO.FALLING, callback=trigger_e_stop, bouncetime=300)
	
	enable_both(False)
	
	print('RasperryPi Pins now Configured....')
	print('IRC5 E Stop Buttons Enabled')
	
def setup_auto_e_stop():
	#Initializes automatic E Stops from the Limit Switches
	
	GPIO.add_event_detect(Sen_L_max, GPIO.FALLING, callback=trigger_L_max, bouncetime=300)
	GPIO.add_event_detect(Sen_R_max, GPIO.FALLING, callback=trigger_R_max, bouncetime=300)
	GPIO.add_event_detect(Sen_L_min, GPIO.FALLING, callback=trigger_L_min, bouncetime=300)
	GPIO.add_event_detect(Sen_R_min, GPIO.FALLING, callback=trigger_R_min, bouncetime=300)
	print('Automatic E Stops from Limit Switches now Enabled')
	
## Motor Code

def left_close(close):
	# sets the direction pin for left motor
	# Parameters:
	# -----------
	# close : Boolean : True if desired direction is towards center
	if close:
		GPIO.output(DIR_L, GPIO.LOW) #TODO fix
	else:
		GPIO.output(DIR_L, GPIO.HIGH)

def right_close(close):
	# sets the direction pin for right motor
	# Parameters:
	# -----------
	# close : Boolean : True if desired direction is towards center
	if close:
		GPIO.output(DIR_R, GPIO.LOW) #TODO
	else:
		GPIO.output(DIR_R, GPIO.HIGH)

def calibrate_left():
    #Moves left manipulator to the outermost position (defined as where the sensor trips)
	# at a rate of 1 mm/s
	
	global left_pos
	print('Calibrating Left Manipulator....')
	while not GPIO.input(Sen_L_min):
		global ls
		ls.ChangeFrequency(1280)
		left_close(False)
		GPIO.output(EN_L, GPIO.HIGH)
		
	GPIO.output(EN_L,GPIO.LOW)
	left_pos = -2
	move_left(0,10)
	print('Left Manipulator Calibrated')
	
def calibrate_right():
    #Moves left manipulator to the outermost position (defined as where the sensor trips)
	# at a rate of 1 mm/s
	
	global right_pos
	print('Calibrating Right Manipulator....')
	while not GPIO.input(Sen_R_min):
		global rs
		rs.ChangeFrequency(1280)
		right_close(False)
		GPIO.output(EN_R, GPIO.HIGH)
		
	GPIO.output(EN_R,GPIO.LOW)
	left_pos = -2
	move_right(0,10)
	print('Right Manipulator Calibrated')
	
def enable_motor(motor,state):
	# Enables a motor
	# Parameters:
	# -----------
	# state : Bool : Motor is enabled if true
	# motor : String : Indicates left motor if 'L', right motor if 'R'
	
	if motor == 'L':
		pin = EN_L
	else:
		pin = EN_R

	if state:
		GPIO.output(pin,GPIO.HIGH)
	else:
		GPIO.output(pin,GPIO.LOW)

def enable_both(state):
	# Enables both motors
	# Parameters:
	# -----------
	# state : Bool : Motor is enabled if true
	
	if state:
		GPIO.output(EN_L,GPIO.HIGH)
		GPIO.output(EN_R,GPIO.HIGH)
	else:
		GPIO.output(EN_L,GPIO.LOW)
		GPIO.output(EN_R,GPIO.LOW)
	
def move_left(abs_pos, speed):	
	# Moves the left manipulator (blocking function)
	# Parameters:
	# -----------
	# abs_pos : Distance from Max Open in Millimeters
	# speed : rate of moving the manipulator in mm/s
	
	global left_pos
	frequency = steps_per_mm * speed
	
	L_to_move = abs_pos - left_pos
	if(L_to_move > 0):
		left_close(True)
	else:
		left_close(False)
		
	global ls
	ls.ChangeFrequency(frequency)

	
	run_time = L_to_move/speed
	end_time = time.time() + run_time
	while(time.time() < end_time):
		enable_motor('L',True)
	enable_motor('L',False)
	left_pos = abs_pos
	
def move_right(abs_pos, speed):	
	# Moves the right manipulator (blocking function)
	# Parameters:
	# -----------
	# abs_pos : Distance from Max Open in Millimeters
	# speed : rate of moving the manipulator in mm/s
	
	global right_pos
	frequency = steps_per_mm * speed
	
	R_to_move = abs_pos - right_pos
	if(R_to_move > 0):
		right_close(True)
	else:
		right_close(False)
		
	global rs
	rs.ChangeFrequency(1280)

	
	run_time = R_to_move/speed
	end_time = time.time() + run_time
	while(time.time() < end_time):
		enable_motor('R',True)
	enable_motor('R',False)
	right_pos = abs_pos
	
def move_both(l_pos, l_speed, r_pos, r_speed):
	# Moves both manipulators (blocking function)
	# Parameters:
	# -----------
	# l_pos : Left Manipulator Distance from Max Open in Millimeters
	# l_speed : rate of moving the left manipulator in mm/s
	# r_pos : right Manipulator Distance from Max Open in Millimeters
	# r_speed : rate of moving the right manipulator in mm/s
	f_l = steps_per_mm * l_speed
	f_r = steps_per_mm * r_speed
	
	global left_pos
	global right_pos
	L_to_move = l_pos - left_pos
	R_to_move = r_pos - right_pos
	
	if(L_to_move > 0):
		left_close(True)
	else:
		left_close(False)
	if(R_to_move > 0):
		right_close(True)
	else:
		right_close(False)
		
	global ls
	global rs
	ls.ChangeFrequency(f_l)
	rs.ChangeFrequency(f_r)
	
	running_L = True
	running_R = True
		
	run_time_L = L_to_move/l_speed
	run_time_R = R_to_move/r_speed
	end_time_L = time.time() + run_time_L
	end_time_R = time.time() + rune_time_R
	enable_both(True)
	
	while running_L or running_R:
		if(time.time() > t_end_L):
			enable_motor('L',False)
			running_L = False
			left_pos = l_pos
		if(time.time() > t_end_R):
			enable_motor('R',False)
			running_R = False
			right_pos = r_pos
	
	
	
def zero_both():
	# zeros both manipulators (blocking function)
	#  disables automatic e stop temporarily
	global ls
	global rs
	
	move_both(5,20,5,20)
	
	zeroing = True
	while not GPIO.input(Sen_L_min):
		ls.ChangeFrequency(1280)
		left_close(False)
		GPIO.output(EN_L, GPIO.HIGH)
		
	GPIO.output(EN_L,GPIO.LOW)
	left_pos = 0
	
	while not GPIO.input(Sen_R_min):
		rs.ChangeFrequency(1280)
		right_close(False)
		GPIO.output(EN_R, GPIO.HIGH)
		
	GPIO.output(EN_R,GPIO.LOW)
	right_pos = 0
	
	zeroing = False
	

		
	

##################################################################


#Actual run stuff
try:
	#Setup
    
	setup_info()
	if not autorun:
		input("Press Enter to continue...")
	setup()
	
	GPIO.output(EN_L, GPIO.LOW)
	GPIO.output(EN_R, GPIO.LOW)
	ls.start(duty_cycle)
	rs.start(duty_cycle)
	ls.ChangeFrequency(250)
	rs.ChangeFrequency(250)
	
	if not autorun:
		print('Calibrating manipulators. Stand Clear\n')
		input("Press Enter to Continue...")

	calibrate_left()
	#calibrate_right()
	
	time.sleep(1)
	setup_auto_e_stop()
	
    #Main Loop
	while True:
		assert e_stop_triggered == False
		#move things accordingly
		#Do stuff
		#do more stuff
except AssertionError:
	Print('Emergency Stop Detected, Program Terminated')
	GPIO.cleanup()
finally:
	GPIO.output(EN_L,GPIO.LOW)
	GPIO.output(EN_R,GPIO.LOW)
	ls.stop()
	rs.stop()
	GPIO.cleanup()
	
	
##TODO: Remove relative positioning for absolute
##TODO: Add speed control (mm/s)
##ROS: Publish current position
##Move pin declarations and functions in different file

