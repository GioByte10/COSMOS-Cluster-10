import picamera
import picamera.array
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

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

pwm_frequency = 50

pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

pwmA.start(0)
pwmB.start(0)

blue_lowerColorThreshold = np.array([20, 65, 67])
blue_upperColorThreshold = np.array([128, 198, 196])

camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
camera.vflip = True
camera.hflip = True

enabled = True
cw = False
maxSpeed = 65


def speedfwd(r):
    a = (1 - r) * maxSpeed
    print(a)
    return a


def drive(r):
    global n

    left_v = 0
    right_v = 0

    left = 0
    right = 0

    for i in range(n // 2 + 1):
        left_v += r[n // 2 + i] * (i + 1) ** 2
        right_v += r[n // 2 - i] * (i + 1) ** 2

    print(left_v, right_v)

    if left_v > right_v:
        cw = True
        mag = left_v

    else:
        cw = False
        mag = right_v

    left = left_v / mag
    right = right_v / mag

    left *= maxSpeed
    right *= maxSpeed

    left = max(left, 30 - right_v)
    right = max(right, 30 - left_v)

    # ~ left += left_v / 2
    # ~ right += right_v / 2

    print(mag, left, right)

    drive_motors(left, right)


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


n = 7
sections = [0] * n
ratios = [0] * n

rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))

try:
    time.sleep(0.1)

    for frame in camera.capture_continuous(rawframe, format='bgr', use_video_port=True):
        image = frame.array
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        ourmask = cv2.inRange(image_hsv, blue_lowerColorThreshold, blue_upperColorThreshold)
        numpixels = cv2.countNonZero(ourmask)
        [numx, numy] = np.shape(ourmask)

        if numpixels != 0:

            for i in range(n):
                sections[i] = ourmask[:, i * numy // n: (i + 1) * numy // n]
                ratios[i] = cv2.countNonZero(sections[i]) / numpixels

            # print(ratios)

            if enabled:
                drive(ratios)

            image_masked = cv2.bitwise_and(image, image, mask=ourmask)
            cv2.imshow("Masked image", image_masked)
            cv2.waitKey(1)

        elif enabled:
            if cw:
                drive_motors(30, -30)

            else:
                drive_motors(-30, 30)

        else:
            drive_motors(0, 0)

        rawframe.truncate(0)

except KeyboardInterrupt:
    pwmA.stop()
    pwmB.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    camera.close()
