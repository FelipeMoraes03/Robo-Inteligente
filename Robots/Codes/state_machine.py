import RPi.GPIO as GPIO
from time import sleep
from pins import *
import math
import threading
from lidar_class import *

# PARAMETROS DO ROBO -> PRECISA SER VERIFICADO NA HORA DE RODAR
THRESHOLD_CURV = 0.5
THRESHOLD_FRONT = 0.3
VEL_PWM_L = 50
VEL_PWM_R = VEL_PWM_L*1.4

# SETUP DOS PINS
GPIO.setmode(GPIO.BCM)
GPIO.setup(RIN1,GPIO.OUT)
GPIO.setup(RIN2,GPIO.OUT)
GPIO.setup(RENA,GPIO.OUT)
GPIO.setup(LIN1,GPIO.OUT)
GPIO.setup(LIN2,GPIO.OUT)
GPIO.setup(LENA,GPIO.OUT)

# SETUP DO PWM
PWM_R=GPIO.PWM(RENA,VEL_PWM_R)
PWM_L=GPIO.PWM(LENA,VEL_PWM_L)

# THREADS
run = True
lock = threading.Lock()
dist_front = None

def lidar_thread():
    # INICIA O LIDAR
    lidar = LidarLoop()
    global dist_front
    global diff_dist

    while run:
        with lock:
            dist_front, diff_dist = lidar.get_diff()

def stop():
    PWM_R.ChangeDutyCycle(0)
    PWM_L.ChangeDutyCycle(0)

def turn_right():
    GPIO.output(LIN1,GPIO.LOW)
    GPIO.output(LIN2,GPIO.HIGH)
    GPIO.output(RIN1,GPIO.HIGH)
    GPIO.output(RIN2,GPIO.LOW)
    PWM_R.ChangeDutyCycle(VEL_PWM_R)
    PWM_L.ChangeDutyCycle(VEL_PWM_L)

def turn_left():
    GPIO.output(LIN1,GPIO.HIGH)
    GPIO.output(LIN2,GPIO.LOW)
    GPIO.output(RIN1,GPIO.LOW)
    GPIO.output(RIN2,GPIO.HIGH)
    PWM_R.ChangeDutyCycle(VEL_PWM_R)
    PWM_L.ChangeDutyCycle(VEL_PWM_L)

def drive_front():
    GPIO.output(LIN1,GPIO.HIGH)
    GPIO.output(LIN2,GPIO.LOW)
    GPIO.output(RIN1,GPIO.HIGH)
    GPIO.output(RIN2,GPIO.LOW)
    PWM_R.ChangeDutyCycle(VEL_PWM_R)
    PWM_L.ChangeDutyCycle(VEL_PWM_L)
    sleep(0.5)

def ajust_right():
    PWM_L.ChangeDutyCycle(0)
    sleep(0.03)
    PWM_L.ChangeDutyCycle(VEL_PWM_L)

def ajust_left():
    PWM_R.ChangeDutyCycle(0)
    sleep(0.03)
    PWM_R.ChangeDutyCycle(VEL_PWM_R)

try:
    # ESTADO INICIAL
    counter = 0
    state = 0

    # INICIA COM PINS SETADOS PARA FRENTE
    GPIO.output(RIN1,GPIO.HIGH)
    GPIO.output(RIN2,GPIO.LOW)
    GPIO.output(LIN1,GPIO.HIGH)
    GPIO.output(LIN2,GPIO.LOW)
    PWM_R.start(0)
    PWM_L.start(0)

    # INICIA A THREAD DO LIDAR
    thread = threading.Thread(target=lidar_thread)
    thread.start()

    # STATE MACHINE
    while run:
        if (dist_front):
            # ESTADO INICIAL
            if (state == 0):
                if counter > 0:
                    state = 2
            
            counter += 1
            print(f"State: {state}, Counter: {counter}")
            print(f"Front: {dist_front}")
            print(f"Diff: {diff_dist}")
            print("")

        # ESTADO PARADA
        if (state == 1):
            stop()
            if (dist_front >= THRESHOLD_FRONT):
                print("FRONT -> 2")
                state = 2
            elif (diff_dist < -THRESHOLD_CURV):
                print("TURN RIGHT -> 4")
                state = 4
            elif (diff_dist >= THRESHOLD_CURV):
                print("TURN LEFT -> 3")
                state = 3
            sleep(0.1)             

        # ESTADO FRENTE
        elif (state == 2):
            state = 5
            if (dist_front < THRESHOLD_FRONT):
                state = 1
            elif (diff_dist >= THRESHOLD_CURV):
                print("AJUST RIGHT -> 2")
                ajust_right()
            elif (diff_dist < -THRESHOLD_CURV):
                print("AJUST LEFT -> 2")
                ajust_left()
            else:
                drive_front()

        # ESTADO ESQUERDA
        elif (state == 3):
            print("STOP -> 1")
            turn_left()
            state = 1

        # ESTADO DIREITA
        elif (state == 4):
            print("STOP -> 1")
            turn_right()
            state = 1

        elif(state == 5):
            sleep(0.1)
            stop()
            sleep(0.5)
            state = 1

        sleep(0.02)

# ENCERRA EXECUCAO
except KeyboardInterrupt:
    run = False
finally:
    thread.join()
    PWM_R.stop()
    PWM_L.stop()
    GPIO.cleanup()