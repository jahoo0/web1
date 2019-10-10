import RPi.GPIO as IO
import time

pwmPin = 26
in3_pin = 20
in4_pin = 21
encPinA = 23
encPinB = 24

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup(pwmPin,IO.OUT)
IO.setup(in3_pin,IO.OUT)
IO.setup(in4_pin,IO.OUT)

p = IO.PWM(pwmPin,100)
p.start(0)

encoderPos = 0
def encoderA(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos += 1
    else :
        encoderPos -= 1

def encoderB(channel):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1
    else :
        encoderPos += 1

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)        #  

pwm = 30.
IO.output(in4_pin, True)
IO.output(in3_pin, False)       # + direction
p.ChangeDutyCycle(pwm)

rev = 0
ratio = 360./29./12.        # degree/gear_ratio/pulse_numb

start_time = time.time()
time_prev = time.time()
dt = 0.
rpm = 0.
pulse = 0.
error = 0.
error_prev = 0.
de = 0.

targetRPM = 150.
threshold = 5

while True:
    motorDeg = encoderPos * ratio
    dt = time.time() - time_prev
    if dt >= 1:
        time_prev = time.time()
        pulse = encoderPos
        encoderPos = 0
        rpm = (pulse/(29.*12.))/dt * 60
        error = targetRPM - rpm
        de = error - error_prev
        error_prev = error

    if ((rpm != 0)and(encoderPos == 0)):

        if abs(error) <= threshold:
            print('Good')

        elif error > threshold:
            pwm += error/80. - abs(de)/200.

        elif error < -threshold:
            pwm -= abs(error)/80. - abs(de)/200.
            
        p.ChangeDutyCycle(pwm)

    print('rpm = %f pmw = %f err = %f de = %f' %(rpm, pwm, error, de))