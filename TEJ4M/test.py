import io
import logging
import socketserver
import argparse
import struct
import sys
import time
import traceback
from http import server
from threading import Condition, Thread
import pigpio
from nrf24 import *
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

PAGE = """
<html>
<head>
<title>picamera2 MJPEG streaming demo</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="1000" height="800" />
</body>
</html>
"""

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning('Removed streaming client %s: %s', self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

def start_streaming_server():
    global output
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (1000, 800)}))
    output = StreamingOutput()
    picam2.start_recording(JpegEncoder(), FileOutput(output))

    address = ('10.0.0.200', 7123)
    server = StreamingServer(address, StreamingHandler)
    print("Starting MJPEG streaming server...")
    try:
        server.serve_forever()
    finally:
        picam2.stop_recording()

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

def start_nrf_receiver():
    print("Python NRF24 Simple Receiver Example.")
    
    # Parse command line arguments.
    parser = argparse.ArgumentParser(prog="simple-receiver.py", description="Simple NRF24 Receiver Example.")
    parser.add_argument('-n', '--hostname', type=str, default='localhost', help="Hostname for the Raspberry running the pigpio daemon.")
    parser.add_argument('-p', '--port', type=int, default=8888, help="Port number of the pigpio daemon.")
    parser.add_argument('address', type=str, nargs='?', default='1SNSR', help="Address to listen to (3 to 5 ASCII characters)")
    
    args = parser.parse_args()
    hostname = args.hostname
    port = args.port
    address = args.address

    if not (2 < len(address) < 6):
        print(f'Invalid address {address}. Addresses must be between 3 and 5 ASCII characters.')
        sys.exit(1)
    
    pi = pigpio.pi(hostname, port)
    if not pi.connected:
        print("Not connected to Raspberry Pi ... goodbye.")
        sys.exit()

    escPin = 2  # GPIO Pin for ESC (Use PWM-capable GPIO)
    pi.set_mode(escPin, pigpio.OUTPUT)

    steeringPin = 18  # GPIO Pin for steering (Use PWM-capable GPIO)
    pi.set_mode(steeringPin, pigpio.OUTPUT)

    nrf = NRF24(pi, ce=12, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.MIN)
    nrf.set_address_bytes(len(address))
    nrf.open_reading_pipe(RF24_RX_ADDR.P1, address.encode('utf-8'))
    nrf.show_registers()

    try:
        print(f'Receive from {address}')
        prevDataTime = 0
        last_data_time = time.time()

        while True:
            if nrf.data_ready():
                pipe = nrf.data_pipe()
                payload = nrf.get_payload()

                if len(payload) == 9 and payload[0] == 0x01:
                    values = struct.unpack("<BLhh", payload)
                    if prevDataTime < values[1]:
                        print(f'Protocol: {values[0]}, time: {values[1]}, x: {values[2]}, y: {values[3]}')
                        pi.set_servo_pulsewidth(escPin, values[3])  # Control ESC
                        set_servo_pulsewidth(pi, steeringPin, values[2])  # Control steering servo
                    else:
                        print("Data outdated")
                    prevDataTime = values[1]

                last_data_time = time.time()
            elif time.time() - last_data_time > 0.25:
                print("No data received")
                last_data_time = time.time()
                pi.set_servo_pulsewidth(escPin, 1000)
                set_servo_pulsewidth(pi, steeringPin, 85)  # Neutral steering position
                
    except:
        traceback.print_exc()
        nrf.power_down()
        pi.stop()

if __name__ == "__main__":
    # Start MJPEG server and NRF receiver in separate threads
    streaming_thread = Thread(target=start_streaming_server)
    nrf_thread = Thread(target=start_nrf_receiver)

    streaming_thread.start()
    nrf_thread.start()

    streaming_thread.join()
    nrf_thread.join()
