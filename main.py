from machine import Pin, PWM
import time
import micropython
#micropython.alloc_emergency_exception_buf(100)

machine.freq(250_000_000)

#CHANGE HERE
path = "SFFT"

use_time_adj = False
use_encoder_adj = True
target_time = 50.0

# action state / speed
index = 0;
base_speed = 50000.0 # 0-65535, dont ask.
speed_A = base_speed # 0-65535, dont ask.
speed_B = base_speed
min_speed = 00000.0;
max_speed = 65535.0;


# angle PID
kp_angle = 80.0
error_angle = 0.0

# time PID
kp_time = 0.0005;
error_time = 0.0;
start_time = 0.0;

# distances
counts_per_forward = 400*75/50;
counts_per_turn = 190;

# encoder ticks
counts_A = 0; # right motor
counts_B = 0; # left motor
target_counts = 0;
total_counts = 0;

# Define motor control pins
PWMA = PWM(Pin(22))  # Motor A PWM
AI1 = Pin(21, Pin.OUT)
AI2 = Pin(20, Pin.OUT)
PWMB = PWM(Pin(15))  # Motor B PWM
BI1 = Pin(13, Pin.OUT)
BI2 = Pin(14, Pin.OUT)
STBY = Pin(12, Pin.OUT)  # Standby pin

# Define encoder pins
MOTOR_A_ENC_A = Pin(7, Pin.IN, Pin.PULL_UP)
MOTOR_A_ENC_B = Pin(8, Pin.IN, Pin.PULL_UP)
MOTOR_B_ENC_A = Pin(11, Pin.IN, Pin.PULL_UP)
MOTOR_B_ENC_B = Pin(10, Pin.IN, Pin.PULL_UP)

# Initialize PWM frequencies
PWMA.freq(1000)
PWMB.freq(1000)

# Interrupt handlers for encoders
def encoder_A_handler(pin):
    global counts_A
    if MOTOR_A_ENC_B.value():
        counts_A += 1
    else:
        counts_A -= 1

def encoder_B_handler(pin):
    global counts_B
    if MOTOR_B_ENC_B.value():
        counts_B += 1
    else:
        counts_B -= 1

# Attach interrupts to encoder channels
MOTOR_A_ENC_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_A_handler)
MOTOR_B_ENC_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_B_handler)

# Function to control motor direction and speed
def setMotor(motor, speed):
    motor_speed = max(min(abs(int(speed)), max_speed), min_speed)
    if motor == "A":
        if speed > 0:
            AI1.value(0)
            AI2.value(1)
        else:
            AI1.value(1)
            AI2.value(0)
        PWMA.duty_u16(motor_speed)
    elif motor == "B":
        if speed > 0:
            BI1.value(0)
            BI2.value(1)
        else:
            BI1.value(1)
            BI2.value(0)
        PWMB.duty_u16(motor_speed)

def stopMotor(motor):
    if motor == "A":
        AI1.value(1)
        AI2.value(1)
        PWMA.duty_u16(0)
    elif motor == "B":
        BI1.value(1)
        BI2.value(1)
        PWMB.duty_u16(0)

# calculate target counts
for a in path:
    if a=="S": # start
        target_counts += counts_per_forward/2
    elif a=="F" or a=="B": # forward or backward
        target_counts += counts_per_forward
    elif a=="R" or a=="L":
        target_counts += counts_per_turn

#print("target counts", target_counts)

# settings
if not use_time_adj:
    kp_time = 0.0;
if not use_encoder_adj:
    kp_angle = 0.0;

time.sleep(1)
STBY.value(1)
start_time = time.ticks_ms()

def adjustSpeeds():
    global error_angle
    global error_time
    global counts_A
    global counts_B
    global speed_A
    global speed_B
    #print("counts:", total_counts + max(abs(counts_A), abs(counts_B)))
    error_angle = abs(counts_A)-abs(counts_B)
    #print(kp_angle*error_angle)
    error_time = (total_counts + max(abs(counts_A), abs(counts_B)))/target_counts*target_time*1000 - (time.ticks_ms()-start_time)
    adj_time = max(min(1-kp_time*error_time, 2), 0.1)
    speed_A = max(min(adj_time*(base_speed - kp_angle*error_angle), max_speed), min_speed)
    speed_B = max(min(adj_time*(base_speed + kp_angle*error_angle), max_speed), min_speed)

while True:
    #print(index, counts_A, speed_A, counts_B, speed_B)
    #print(counts_A, counts_B, speed_A, speed_B)
    if path[index]=="S":
        #print("start")
        if max(counts_A, counts_B) > counts_per_forward/2:
            total_counts += counts_per_forward/2
            counts_A -= counts_per_forward/2
            counts_B -= counts_per_forward/2
            index += 1
            #print("end start")
        else:
            adjustSpeeds()
            setMotor("A", speed_A)
            setMotor("B", speed_B)
    elif path[index]=="F":
        #print("forward", counts_A, counts_B)
        if max(counts_A, counts_B) > counts_per_forward:
            print(counts_A, speed_A, counts_B, speed_B)
            total_counts += counts_per_forward
            counts_A -= counts_per_forward
            counts_B -= counts_per_forward
            index += 1
            #print("end forward")
        else:
            adjustSpeeds()
            setMotor("A", speed_A)
            setMotor("B", speed_B)
    elif path[index]=="B":
        if max(-counts_A, -counts_B) > counts_per_forward:
            total_counts += counts_per_forward
            counts_A += counts_per_forward
            counts_B += counts_per_forward
            index += 1
        else:
            adjustSpeeds()
            setMotor("A", -speed_A)
            setMotor("B", -speed_B)
    elif path[index]=="R":
        if max(-counts_A, counts_B) > counts_per_turn:
            total_counts += counts_per_turn
            counts_A += counts_per_turn
            counts_B -= counts_per_turn
            index += 1
        else:
            adjustSpeeds()
            setMotor("A", -speed_A)
            setMotor("B", speed_B)
    elif path[index]=="L":
        if max(counts_A, -counts_B) > counts_per_turn:
            total_counts += counts_per_turn
            counts_A -= counts_per_turn
            counts_B += counts_per_turn
            index += 1
        else:
            adjustSpeeds()
            setMotor("A", speed_A)
            setMotor("B", -speed_B)
    elif path[index]=="T": # terminate
        stopMotor("A")
        stopMotor("B")
        break


