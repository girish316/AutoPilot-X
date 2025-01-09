import struct
import sys
import traceback
import cv2
import numpy as np
from picamera2 import Picamera2
import utlis
import time
import pigpio
from simple_pid import PID
from threading import Thread
import keyboard  # Keyboard input for failsafe
from nrf24 import *

# Initialize pigpio for PWM control
pi = pigpio.pi()
ESC_PIN = 2  # GPIO pin for ESC
STEERING_PIN = 18  # GPIO pin for steering
pi.set_mode(ESC_PIN, pigpio.OUTPUT)
pi.set_mode(STEERING_PIN, pigpio.OUTPUT)

# PID setup
Kp, Ki, Kd = 0.35, 0, 0
pid = PID(Kp, Ki, Kd, setpoint=0)
pid.sample_time = 0.05  # 50 ms sample time
pid.output_limits = (-25, 25)  # Steering correction limits in degrees

curveList = []
avgVal = 10
lane_center = 240  # Initial lane center (adjustable)

tolerance = 5  # Tolerance in pixels for acceptable deviation


def set_servo_pulsewidth(pi, servo_pin, angle):
    """
    Adjust pulse width based on asymmetrical steering needs.
    Args:
    - pi: pigpio instance
    - servo_pin: GPIO pin connected to the servo
    - angle: Desired angle (0 to 180 degrees)
    """
    # Apply offset for joystick error (center at 85 instead of 90)
    corrected_angle = angle - 35  # Adjusting center to 85
    
    # Asymmetric pulse width mapping: increase right turn range
    if corrected_angle < 90:
        # Scale right turn range more aggressively (lesser than 65)
        pulsewidth = (corrected_angle / 90.0) * 600 + 1000  # Range: 1000 to 1600
    else:
        # Scale left turn range normally
        pulsewidth = ((corrected_angle - 90) / 90.0) * 400 + 1600  # Range: 1600 to 2000

    # Clamp the pulse width to ensure it stays within servo limits
    pulsewidth = max(1000, min(2000, pulsewidth))
    
    pi.set_servo_pulsewidth(servo_pin, pulsewidth)
def steer_with_pid(pi, servo_pin, current_deviation):
    """
    Use PID to compute steering correction and apply it using the manual control function.
    Args:
    - pi: pigpio instance
    - servo_pin: GPIO pin connected to the servo
    - current_deviation: Deviation from the center of the lane
    """
    if abs(current_deviation) > tolerance:
        correction_angle = pid(current_deviation)
        desired_angle = 85 + correction_angle  # Neutral position is 85 degrees
        set_servo_pulsewidth(pi, servo_pin, desired_angle)

def getDeviation(img, display):
    global lane_center

    imgCopy = img.copy()
    imgResult = img.copy()

    #### STEP 1: Thresholding the image
    imgThres = utlis.thresholding(img)

    #### STEP 2: Warp the image
    hT, wT, c = img.shape
    points = utlis.valTrackbars()
    imgWarp = utlis.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utlis.drawPoints(imgCopy, points)

    middlePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.5, region=4)
    curveAveragePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    if display != 0:
        imgInvWarp = utlis.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0

        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)

        row_y = 200  # Row of interest
        green_color = [0, 255, 0]  # Green color in BGR
        row_pixels = imgLaneColor[row_y, :, :]  # Extract the row 200 pixels (all columns, row 200)
        colored_pixels = np.where(np.all(row_pixels == green_color, axis=-1))[0]  # Find where the row pixels are equal to green color

        if len(colored_pixels) > 0:
            min_x = colored_pixels[0]  # First column with green color
            max_x = colored_pixels[-1]  # Last column with green color
            lane_center = (min_x + max_x) / 2.0  # Calculate the lane center


        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)

        midY = 450
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)

    curve = curve / 100
    if curve > 1:
        curve = 1
    if curve < -1:
        curve = -1

    deviation = lane_center - ((wT /2)+15)

    # Display the final result
    if display == 2:
        imgStacked = utlis.stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        return imgStacked
    elif display == 1:

        text = str(deviation)
        (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_ITALIC, 1.5, 3)
        x_pos = wT // 2 - text_width // 2
        cv2.putText(imgResult, text, (x_pos, 85), cv2.FONT_ITALIC, 1.5, (255, 0, 255), 3)

        midY = 200  # Y-coordinate where the lane is potentially useful for calculations
        cv2.circle(imgResult, (int(wT/2), midY), 10, (0, 0, 255), -1)  # Red circle at predicted position
        cv2.circle(imgResult, (int(lane_center), midY), 10, (255, 0, 0), -1)  # Blue circle at Lane Center  

        # Show the result
        cv2.imshow('Result', imgResult)

    return deviation

def calculateSteeringAdjustment(curve):
    global lane_center
    steeringAdjustment = lane_center + curve * 50  # Adjust the steering prediction based on curve
    return steeringAdjustment


def start_nrf_receiver():
    """
    NRF receiver for manual ESC control.
    """
    print("Starting NRF receiver...")
    nrf = NRF24(pi, ce=12, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.MIN)
    nrf.set_address_bytes(5)
    nrf.open_reading_pipe(RF24_RX_ADDR.P1, b'1SNSR')
    nrf.show_registers()

    last_data_time = time.time()

    try:
        while True:
            if nrf.data_ready():
                pipe = nrf.data_pipe()
                payload = nrf.get_payload()

                if len(payload) == 9 and payload[0] == 0x01:
                    values = struct.unpack("<BLhh", payload)
                    print(f'Protocol: {values[0]}, time: {values[1]}, x: {values[2]}, y: {values[3]}')

                    speed = (values[3] - 1000)/2 + 1000
                    pi.set_servo_pulsewidth(ESC_PIN, speed)  # Control ESC
                    last_data_time = time.time()

            elif time.time() - last_data_time > 0.25:
                print("No data received, stopping ESC.")
                pi.set_servo_pulsewidth(ESC_PIN, 1000)  # Neutral ESC
    except Exception as e:
        print(f"Error in NRF receiver: {e}")
        traceback.print_exc()
    finally:
        nrf.power_down()
        pi.stop()

def keyboard_failsafe():
    """
    Keyboard failsafe to stop the script and all PWM outputs.
    """
    print("Press 'q' to stop the program immediately.")
    while True:
        if keyboard.is_pressed('q'):
            print("Failsafe activated. Stopping all PWM outputs.")
            pi.set_servo_pulsewidth(STEERING_PIN, 0)  # Turn off steering
            pi.set_servo_pulsewidth(ESC_PIN, 0)  # Turn off ESC

            pi.stop()
            sys.exit(0)


if __name__ == '__main__':
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (480, 240)}))
    picam2.start()

    initialTrackBarVals = [120, 127, 20, 240]
    utlis.initializeTrackbars(initialTrackBarVals)

    # Start keyboard failsafe in a separate thread
    failsafe_thread = Thread(target=keyboard_failsafe)
    nrf_thread = Thread(target=start_nrf_receiver)
    nrf_thread.start()
    failsafe_thread.start()

    try:
        while True:
            frame = picam2.capture_array()
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            deviation = getDeviation(img, display=1)  # Get lane deviation
            steer_with_pid(pi, STEERING_PIN, deviation)  # Apply PID-based steering

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        picam2.stop()
        pi.set_servo_pulsewidth(STEERING_PIN, 0)
        pi.set_servo_pulsewidth(ESC_PIN, 0)
        cv2.destroyAllWindows()
