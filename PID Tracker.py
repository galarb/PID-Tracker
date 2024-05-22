#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
gyro_sensor_in3 = GyroSensor(INPUT_3)
gps_sensor_in4 = GPSSensor(INPUT_4)
pen_in5 = Pen(INPUT_5)
ultrasonic_sensor_in6 = UltrasonicSensor(INPUT_6)
ultrasonic_sensor_in7 = UltrasonicSensor(INPUT_7)
#ultrasonic_sensor_in8 = UltrasonicSensor(INPUT_8)
camera_sensor_in1 = CameraSensor(INPUT_8)
motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts
debugval = 2000 #the rateError value that triggeres the debuggin 
error = None
previousTime = time.time() * 1000
cumError = 0  # Initialize cumulative error
lastError = 0  # Initialize last error
rateError = 0
seconddev = 0
rlF_pv = 0
flag = False
lastdev = 0
error_debug = []
pid_values_debug=[]
rateError_debug =[]
definiteang =0
cycle = 25#determin the Ti reset time
# Describe this function...
def pidcalc(sp, pv, Kp, Ki, Kd, Ks):

    global error, previousTime, cumError, lastError, rateError, lastdev, flag, seconddev, cycle  # Specify global variables
    currentTime = int(time.time() * 1000)             
    elapsedTime = (currentTime - previousTime)/1000
    #print('elapsed time =', elapsedTime)
    error = sp - pv
    rateError = (error - lastError) / elapsedTime
    seconddev = (rateError - lastdev) / elapsedTime
    if not flag: #error is still accomulating, did not change direction yet
        cumError += error * elapsedTime
        
    else: #error has changed direction
        cumError = 0  # Reset cumulative error and rate error if elapsedTime is zero
        #print("resetting Integral value")
   
    #print('cumError,', int(cumError * Ki), ',rateError,', int(rateError) * Kd, ',seconddev,', int(seconddev), ',flag,', flag, end='')

    out = Kp*error + Ki*cumError + Kd*rateError + Ks*seconddev
    previousTime = currentTime
    #to find the flip point, where error changes direction. 
    if (rateError < 2 and rateError > -2):
        flag = True
    else:
        flag = False
        #print('resetting flag')

    lasterror = error
    '''if abs(rateError) > debugval: 
        debug(out, rateError, Ki, Kd)'''
    
    lastrlF_pv = rlF_pv
    lastdev = rateError
    error_debug.append(error)
    rateError_debug.append(rateError)

    #else: debug(0,0,0)
    return out
    
def debug(out, rateError, Ki, Kd):
    pen_in5.setColor(0.5, 0, 0.95)
    pen_in5.setWidth(2)
    #print('out =', out, 'rateError =', rateError, 'Ki =', Ki, 'Kd = ', Kd)
    pen_in5.down()

def steer(ang_sp, speed, Kp):
    ang_pv = gyro_sensor_in3.angle
    delta = pidcalc(ang_sp, ang_pv, Kp, 0, 0, 0)
    speedR = speed - delta
    speedL = speed + delta
    if speedR > 100:
        speedR = 100
    if speedL > 100:
        speedL = 100
    if speedR < -100:
        speedR = -100
    if speedL < -100:
        speedL = -100
    #print("speedR =", speedR, "speeedL=", speedL)
    #print("delta =", delta)
    tank_drive.on(speedL, speedR)

# track reflected_light_intensity. for how many itterations
def track(speed, times):
    reflected_light_sp = 50 #track mid line 
    for i in range(times):
        rl_pv = color_sensor_in1.reflected_light_intensity # 100 = white, 0 = black

        delta = pidcalc(reflected_light_sp, rl_pv, 1, 1.1, 0.02, 0.00001) #PID and a Ks for secondary deviation
 
        #print(',itteration number,', i, ',reflected light reading,', rl_pv, ',delta,', int(delta), end='')
        #print(int(delta))
        if delta > 0 :
            #print('going left')
            speedR = speed + delta
            speedL = speed - delta
        elif delta < 0:
            #print('going right')
            speedR = speed - abs(delta)
            speedL = speed + abs(delta)
        else:
            #print('going stright')
            speedR = speed
            speedL = speed
        
        
        if speedR > 100:
            speedR = 100
        if speedL > 100:
            speedL = 100
        if speedR < -100:
            speedR = -100
        if speedL < -100:
            speedL = -100

        checkdirection()
        tank_drive.on(speedL, speedR)
        #print(',speedR,', int(speedR), ',speedL,', int(speedL))
    
def checkdirection():
    ang_pv = gyro_sensor_in3.angle
    if(ang_pv > 360):
        turn (0, 20, 1)
        steer(0, 20, 1)

        

def turn(ang_sp, times, Kp):
    print ("turning to ", ang_sp)
    for i in range(times):
        ang_pv = gyro_sensor_in3.angle
        speedR = -pidcalc(ang_sp, ang_pv, Kp, 0, 0, 0)
        speedL = -speedR
       
        if speedR > 100:
            speedR = 100
        elif speedR < -100:
            speedR = -100
        if speedL > 100:
            speedL = 100
        elif speedL < -100:
            speedL = -100
        tank_drive.on(speedL, speedR)
        # Update ang_pv within the loop
        ang_pv = gyro_sensor_in3.angle
    #print("finished turning", ang_pv)

track(25, 1000)#default speed, how many iterations

tank_drive.off(brake=True)