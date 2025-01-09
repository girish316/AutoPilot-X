from raspiSensors import hcsr04 as HCSR04 #     pip install raspiSensors


leftUS, rightUS, frontUS = HCSR04(trigger_pin = 21, echo_pin = 20), HCSR04(trigger_pin = 18, echo_pin = 19), HCSR04(trigger_pin = 16, echo_pin = 17)

print(ultrasonic.distance())

