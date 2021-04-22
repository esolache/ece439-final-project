#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Servo Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-16

servo_node.py
ROS Node to set servo pulses received on input topics (e.g. "/servo_command_0"). 
"""
import rospy
import traceback 
# Import the Servo library: 
import Adafruit_PCA9685      ## Alternative library, different function: import pi_servo_hat
# IMPORT the messages: 
from std_msgs.msg import Int32

# Set up scaling parameters for pulse width
pulseFreq = 60 
pulseLength = 1000000/pulseFreq/4096                   # 1,000,000 us per second, 60 Hz, 12 bits (=4096 levels)
#    print("{} us per bit".format(pulseLength))

# Set up the PCA9685 PWM chip to command the servos. 
pwm = Adafruit_PCA9685.PCA9685()           # Initialise the PWM device using the default address
pwm.set_pwm_freq(pulseFreq)                        # Set frequency to 60 Hz


    


# =============================================================================
#   # Main function
# =============================================================================
def main(): 
    # =============================================================================
    #     # Launch a node called "servo_node"
    # =============================================================================
    rospy.init_node('servo_node', anonymous=False)
    print('Servos Ready!')
    
    # Call the initialization function to ensure a soft start. 
    initialize_pwm() # 
    
    # Set up subscriber that listens to "/servo_command_0"
    sub_servo_command_0 = rospy.Subscriber('/servo_command_0', Int32, command_servo_0)
    
    rospy.spin()
    

# =============================================================================
#   # Callback function: receives a desired joint angle
def command_servo_0(msg_in): 
    # unpack the command
    cmd_0 = msg_in.data
    setServoPulse(0,cmd_0)
    
# Define a Function to set the servo command on a given channel to a specified command 
def setServoPulse(channel, pulse_us):
    # Actually set the command
    pwm.set_pwm(channel, 0, int(pulse_us/pulseLength))
#    print([channel, pulse_us, int(pulse_us/pulseLength)])

# Define a function to shut down all the servos by sending them zero pulse width (same as no command)
def initialize_pwm():
    for ii in range(16):
        setServoPulse(ii, 0)
    


    
    
# Startup stuff. 
if __name__ == '__main__':
    try: 
        main()
    except: 
        traceback.print_exc()
        pass

# Shut down here also, just in case. 
initialize_pwm()  # to shut it down. 
print('Servos Shut Down.')