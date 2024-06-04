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
color_sensor_head = ColorSensor(INPUT_9)

motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts
#change the Kp, Ki and Kd as you will, but make sure to comment the zieglernichols() in pidcalc function

Kp = 1.15
Ki = 3.9
Kd = 0.004
Ks = 0.000

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
elapsedTime = 0.02
printflag = True # rue False

# Describe this function...
def pidcalc(sp, pv):
    
    global error, previousTime, cumError, lastError, rateError, lastdev, flag, seconddev, cycle  # Specify global variables
    currentTime = time.time() * 1000             
    elapsedTime = (currentTime - previousTime)/1000
    #zieglernichols()
    #print('elapsed time =', elapsedTime)
    error = sp - pv
    rateError = (error - lastError) / elapsedTime
    seconddev = (rateError - lastdev) / elapsedTime
    if not flag: #error is still accomulating, did not change direction yet
        cumError += error * elapsedTime
        
    else: #error has changed direction
        cumError = 0  # Reset cumulative error and rate error if elapsedTime is zero
        #print("resetting Integral value")
   
    if printflag:
        print('cumError,', int(cumError * Ki), ',rateError,', int(rateError) * Kd, ',seconddev,', int(seconddev), ',flag,', flag, end='')

    out = Kp*error + Ki*cumError + Kd*rateError + Ks*seconddev
    #print('Kp =', Kp, 'Ki =', Ki, 'Kd =', Kd)

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
    if printflag:
        print('out =', out, 'rateError =', rateError, 'Ki =', Ki, 'Kd = ', Kd)
    pen_in5.down()

def steer(ang_sp, speed, Kp):
    print('steering')
    ang_pv = gyro_sensor_in3.angle
    delta = pidcalc(ang_sp, ang_pv)
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
def findgap():
    pass


def zieglernichols():
    global Kp, Ki, Kd, elapsedTime
    Ku = 2.2#ultimate gain. where stable oscilations occur
    Kp = 0.5*Ku
    
    #Tu - the cycle time of the oscilations, was measured between 18 and 19 itterations
    Tu = 18*elapsedTime
    Ti = 0.8*Tu
    Td = 0.01*Tu
    
    Ki = Kp / Ti
    Kd = Kp*Td
    #print('Kp =', Kp, 'Ki =', Ki, 'Kd =', Kd)
    
    
# track reflected_light_intensity. for how many itterations
def track(speed, times):
    reflected_light_sp = 50 #track mid line 
    for i in range(times):
        if ultrasonic_sensor_in2.distance_centimeters > 10:
            rl_pv = color_sensor_in1.reflected_light_intensity # 100 = white, 0 = black
            rl_pv_head = color_sensor_head.reflected_light_intensity

            delta = pidcalc(reflected_light_sp, rl_pv) #PID and a Ks for secondary deviation
            if printflag:
                print(',itteration number,', i, ',reflected light reading,', rl_pv, ',delta,', int(delta), end='')
            
            if delta > 0 :
                #print('going left')
                speedR = speed + delta
                speedL = speed 
            elif delta < 0:
                #print('going right')
                speedR = speed 
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
    
            checkdirection(rl_pv)
            tank_drive.on(speedL, speedR)
            if printflag:
                print(',speedR,', int(speedR), ',speedL,', int(speedL))
        else: 
            avoidobstacle(40, 200) #speed, times

def avoidobstacle(speed, times):
    turn(90, 40, 1)
    for i in range(times):
       steer(90, speed, 1) 
    turn(0, 40, 1)
    for i in range(1.4*times):
       steer(0, speed, 1) 
    turn(-90, 40, 1)
    for i in range(times):
       steer(-90, speed, 1) 
    turn(0, 40, 1)
    tank_drive.on(25,25)   

def checkdirection(rl_pv):
    ang_pv = gyro_sensor_in3.angle

    if(ang_pv > 360):
        turn (0, 20, 1)
        steer(0, 20, 1)

        

def turn(ang_sp, times, Kp):
    #print ("turning to ", ang_sp)
    for i in range(times):
        ang_pv = gyro_sensor_in3.angle
        speedR = -pidcalc(ang_sp, ang_pv)
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

gyro_sensor_in3.reset()
track(30, 100)#default speed, how many iterations

tank_drive.off(brake=True)