from evdev import InputDevice, categorize
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

kit = ServoKit(channels=16)
channel_servo1 = 6
kit.servo[channel_servo1].set_pulse_width_range(400, 2300)

GPIO_Ain1 = 17
GPIO_Ain2 = 27
GPIO_Apwm = 22
GPIO_Bin1 = 5
GPIO_Bin2 = 6
GPIO_Bpwm = 13

GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 50

# Create the PWM instances
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

pwmA.start(0)
pwmB.start(0)

gamepad = InputDevice('/dev/input/event0')
print(gamepad)

x = 0
y = 1

blue = 0
red = 1
yellow = 2
green = 3
left = 4
right = 5
select = 8
start = 9

joystick = [128, 128]
buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

enabled = True


def stop_execution():
    gamepad.close()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()


def handle_input():
    if newbutton:
        buttons[codebutton - 304] = valuebutton
        print(buttons)

    elif newstick:
        joystick[codestick] = valuestick
        print(joystick)


def drive_motors(left, right):
    if left > 0:
        GPIO.output(GPIO_Ain1, True)
        GPIO.output(GPIO_Ain2, False)

    else:
        GPIO.output(GPIO_Ain1, False)
        GPIO.output(GPIO_Ain2, True)

    if right > 0:
        GPIO.output(GPIO_Bin1, True)
        GPIO.output(GPIO_Bin2, False)

    else:
        GPIO.output(GPIO_Bin1, False)
        GPIO.output(GPIO_Bin2, True)

    pwmA.ChangeDutyCycle(abs(left))
    pwmB.ChangeDutyCycle(abs(right))


def drive():
    if joystick[x] == 128 and joystick[y] == 0:  # forward
        drive_motors(100, 100)

    elif joystick[x] == 128 and joystick[y] == 255:  # backwards
        drive_motors(-100, -100)

    elif joystick[x] == 0 and joystick[y] == 128:  # ccw
        drive_motors(-100, 100)

    elif joystick[x] == 255 and joystick[y] == 128:  # cw
        drive_motors(100, -100)

    elif joystick[x] == 0 and joystick[y] == 0:  # left
        drive_motors(40, 100)

    elif joystick[x] == 255 and joystick[y] == 0:  # right
        drive_motors(100, 40)

    elif joystick[x] == 128 and joystick[y] == 128:  # idle
        drive_motors(0, 0)


def attack():
    kit.servo[channel_servo1].angle = 180 if buttons[red] else 0


# Main code
try:

    noError = True
    while noError:

        newbutton = False
        newstick = False
        try:
            for event in gamepad.read():
                eventinfo = categorize(event)

                if event.type == 1:
                    newbutton = True
                    codebutton = eventinfo.scancode
                    valuebutton = eventinfo.keystate

                elif event.type == 3:
                    newstick = True
                    codestick = eventinfo.event.code
                    valuestick = eventinfo.event.value
        except:
            pass

        handle_input()

        if buttons[left]:
            enabled = not enabled
            print("enabled" if enabled else "disabled")
            time.sleep(0.2)

        if buttons[select]:
            stop_execution()

        if enabled:
            drive()
            attack()

        else:
            pwmA.ChangeDutyCycle(0)
            pwmB.ChangeDutyCycle(0)

except KeyboardInterrupt:
    stop_execution()

