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

Kp = 1.8
Ki = 0.0
Kd = 0.06
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
delta_time = []
start_time = time.time()
error_debug = []
pid_values_debug=[]
rateError_debug =[]
filtered_d_debug = []
last_speedL = 0
last_speedR = 0
alpha = 0.3  # smoothing factor: 0.0 = no change, 1.0 = full new value

definiteang =0
cycle = 25#determin the Ti reset time
elapsedTime = 0.02
printflag = False # True False
filtered_d = 0  # filtered derivative
accumTime = 0   # accumulated runtime in seconds

# Describe this function...
def pidcalc(sp, pv):
    global error, previousTime, cumError, lastError, rateError, lastdev, flag
    global seconddev, cycle, filtered_d, accumTime, alpha, last_speedL, last_speedR

    currentTime = time.time() * 1000
    elapsedTime = (currentTime - previousTime) / 1000

    error = sp - pv
    rateError = (error - lastError) / elapsedTime

    # low-pass filter
    
    filtered_d = alpha * rateError + (1 - alpha) * filtered_d

    # accumulate time
    accumTime += elapsedTime

    # PID output
    out = Kp * error + Ki * cumError + Kd * filtered_d + Ks * seconddev

    previousTime = currentTime
    lastError = error
    lastdev = rateError

    # logging
    delta_time.append(accumTime)
    error_debug.append(error)
    rateError_debug.append(rateError)
    filtered_d_debug.append(filtered_d)

    return out
    
def debug(out, rateError, Ki, Kd):
    pen_in5.setColor(0.5, 0, 0.95)
    pen_in5.setWidth(2)
    if printflag:
        print('out =', out, 'rateError =', rateError, 'Ki =', Ki, 'Kd = ', Kd)
    pen_in5.down()


# track reflected_light_intensity. for how many itterations
def track(speed, times):
    global last_speedL, last_speedR  # needed for filter state
    
    reflected_light_sp = 50  # track mid line
    for i in range(times):
        if ultrasonic_sensor_in2.distance_centimeters > 10:
            rl_pv = color_sensor_in1.reflected_light_intensity

            # PID
            delta = pidcalc(reflected_light_sp, rl_pv)

            # stronger slowdown in corners
            corner_factor = max(0.5, 1 - abs(delta)/40)
            base_speed = speed * corner_factor

            # nonlinear turn gain, capped
            turn_gain = min((abs(delta)/40)**1.3, 0.8)
            if delta > 0:  # turn left
                speedR = base_speed
                speedL = base_speed * (1 - 2 * turn_gain)  # can go negative if turn_gain > 0.5
            elif delta < 0:  # turn right
                speedL = base_speed
                speedR = base_speed * (1 - 2 * turn_gain)
            else:  # straight
                speedR = base_speed
                speedL = base_speed
            

            # clamp speeds
            speedR = max(min(speedR, 100), -100)
            speedL = max(min(speedL, 100), -100)
            
            # --- low-pass filter (corrected) ---
            speedL = (1 - alpha) * speedL + alpha * last_speedL
            speedR = (1 - alpha) * speedR + alpha * last_speedR
            #if abs(delta) > 10:   # threshold → corner condition
                #print(f"[corner] delta={delta:.2f}, speedL={speedL:.2f}, speedR={speedR:.2f}")
            last_speedL = speedL
            last_speedR = speedR

            tank_drive.on(speedL, speedR)

            if printflag:
                print(',speedR,', int(speedR), ',speedL,', int(speedL))
        else: 
            avoidobstacle(40, 200)  # speed, times

track(50, 2000)#default speed, how many iterations
print("time,error,rateError")
#for t, e, r in zip(delta_time, error_debug, rateError_debug):
    #print(f"{t:.3f},{e:.3f},{r:.3f}")
tank_drive.off(brake=True)
