


import RPi.GPIO as GPIO

echo_pins_us_sensors = [10,9,11,0]



GPIO.setmode(GPIO.BCM)
GPIO.setup(echo_pins_us_sensors[0], GPIO.IN)
GPIO.setup(echo_pins_us_sensors[1], GPIO.IN)
GPIO.setup(echo_pins_us_sensors[2], GPIO.IN)
GPIO.setup(echo_pins_us_sensors[3], GPIO.IN)

