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
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)

IO.output(in4_pin, True)
IO.output(in3_pin, False)
p.ChangeDutyCycle(50)

rev = 0
ratio = 360./29./12.

start_time = time.time()
time_prev = 0.
dt = 0.
rpm = 0.

while True:
	motorDeg = encoderPos * ratio

	if encoderPos >= 29*12 :
		encoderPos = 0
		rev += 1
		dt = time.time() - time_prev
		rpm = 1/dt*60
		time_prev = time.time()

		if rev = 10:
			p.ChangeDutyCycle(0)
			IO.cleanup()
			break

	print('Deg = %f pulse = %f rev = %d s = %f rpm = %f' %(motorDeg, encoderPos, rev, dt, rpm))

