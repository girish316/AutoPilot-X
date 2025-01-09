from flask import Flask, Response, render_template, request, jsonify
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
from threading import Condition, Thread, Event
import keyboard
from nrf24 import *

# Initialize Flask app
app = Flask(__name__)

# Global flags for auto mode and emergency stop
auto_mode = Event()  # Auto mode toggle
emergency_stop = Event()  # Emergency stop flag

# Initialize pigpio for PWM control
pi = pigpio.pi()
ESC_PIN = 2
STEERING_PIN = 18
pi.set_mode(ESC_PIN, pigpio.OUTPUT)
pi.set_mode(STEERING_PIN, pigpio.OUTPUT)

# PID setup
Kp, Ki, Kd = 0.35, 0, 0
pid = PID(Kp, Ki, Kd, setpoint=0)
pid.sample_time = 0.05
pid.output_limits = (-25, 25)

tolerance = 5  # Tolerance for deviation
lane_center = 240

curveList = []
avgVal = 10
lane_center = 240  # Initial lane center (adjustable)

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (480, 240), "format": "RGB888"})
picam2.configure(config)
picam2.start()


def set_servo_pulsewidth(pi, servo_pin, angle):
    corrected_angle = angle - 35
    if corrected_angle < 90:
        pulsewidth = (corrected_angle / 90.0) * 600 + 1000
    else:
        pulsewidth = ((corrected_angle - 90) / 90.0) * 400 + 1600
    pulsewidth = max(1000, min(2000, pulsewidth))
    pi.set_servo_pulsewidth(servo_pin, pulsewidth)


def steer_with_pid(pi, servo_pin, current_deviation):
    if abs(current_deviation) > tolerance:
        correction_angle = pid(current_deviation)
        desired_angle = 85 + correction_angle
        set_servo_pulsewidth(pi, servo_pin, desired_angle)

def getDeviation(img, display=1):
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

    deviation = lane_center - ((wT /2)+25)

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


    return deviation, imgResult



@app.route('/stream')
def stream():
    return Response(mjpeg_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')
def mjpeg_stream():
    while True:
        frame = picam2.capture_array()
        deviation, imgResult = getDeviation(frame)
        _, jpeg = cv2.imencode('.jpg', imgResult)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


auto_mode = Event()  # Use threading.Event for auto mode state

@app.route('/control', methods=['POST'])
def control():
    command = request.json.get('command')

    if command == 'enable_auto':
        auto_mode.set()  # Enable auto mode
        response = {"status": "Auto mode enabled"}
    elif command == 'disable_auto':
        auto_mode.clear()  # Disable auto mode
        response = {"status": "Auto mode disabled"}
    elif command == 'emergency_stop':
        # Handle emergency stop logic
        response = {"status": "Emergency stop activated"}
    else:
        response = {"status": "Unknown command"}

    return jsonify(response)

@app.route('/status', methods=['GET'])
def status():
    return jsonify({"auto_mode": auto_mode.is_set()})  # Return current auto mode state

prevDataTime = 0
last_data_time = time.time()

nrf_condition = Condition()

def nrf_receiver():
    global last_data_time, prevDataTime
    nrf = NRF24(pi, ce=12, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.MIN)

    nrf.set_address_bytes(5)
    nrf.open_reading_pipe(RF24_RX_ADDR.P1, b'1SNSR')
    nrf.show_registers()

    while not emergency_stop.is_set():
        with nrf_condition:
            if nrf.data_ready():
                pipe = nrf.data_pipe()
                payload = nrf.get_payload()

                if len(payload) == 9 and payload[0] == 0x01:
                    values = struct.unpack("<BLhh", payload)
                    if prevDataTime < values[1]:
                        print(f'Protocol: {values[0]}, time: {values[1]}, x: {values[2]}, y: {values[3]}')
                        speed = values[3]
                        if not auto_mode.is_set():
                            set_servo_pulsewidth(pi, STEERING_PIN, values[2])
                            pi.set_servo_pulsewidth(ESC_PIN, speed)

                    else:
                        print("Data outdated")
                    last_data_time = time.time()

            elif time.time() - last_data_time > 0.3:
                print("No data received")
                last_data_time = time.time()
                pi.set_servo_pulsewidth(ESC_PIN, 1000)

            # Notify the steering loop that NRF has processed data
            nrf_condition.notify()


def steering_loop():
    auto_mode_active = False  # Flag to track auto mode state

    while not emergency_stop.is_set():
        with nrf_condition:
            nrf_condition.wait()  # Wait until NRF receiver signals

            if auto_mode.is_set():
                # Auto mode is enabled
                frame = picam2.capture_array()
                deviation, _ = getDeviation(frame)
                steer_with_pid(pi, STEERING_PIN, deviation)
                pi.set_servo_pulsewidth(ESC_PIN, 1050)
                auto_mode_active = True  # Update flag
            else:
                # Auto mode is disabled
                if auto_mode_active:  # Check if auto mode was just disabled
                    pi.set_servo_pulsewidth(ESC_PIN, 1000)  # Set ESC pulse width to 1000 once
                    auto_mode_active = False  # Reset flag




# Start NRF receiver and steering loop in separate threads
nrf_thread = Thread(target=nrf_receiver, daemon=True)
steering_thread = Thread(target=steering_loop, daemon=True)
nrf_thread.start()
steering_thread.start()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=7123)
