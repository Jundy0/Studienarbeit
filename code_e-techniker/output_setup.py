
import RPi.GPIO as GPIO

pin_steering_motor = 23
pin_motor_forward = 24
pin_motor_backward = 25

pin_mode_LED_1 = 16
pin_mode_LED_2 = 20

pin_left_LED = 27
pin_right_LED = 22


trigger_pins_us_sensors = [8,7,1,12]




GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_motor_forward, GPIO.OUT)
GPIO.setup(pin_motor_backward, GPIO.OUT)
GPIO.setup(pin_steering_motor, GPIO.OUT)

#GPIO.setup(pin_mode_LED_1, GPIO.OUT)
#GPIO.setup(pin_mode_LED_2, GPIO.OUT)
#GPIO.setup(pin_left_LED, GPIO.OUT)
#GPIO.setup(pin_right_LED, GPIO.OUT)


GPIO.setup(trigger_pins_us_sensors[0], GPIO.OUT)
GPIO.setup(trigger_pins_us_sensors[1], GPIO.OUT)
GPIO.setup(trigger_pins_us_sensors[2], GPIO.OUT)
GPIO.setup(trigger_pins_us_sensors[3], GPIO.OUT)




motor_forward = GPIO.PWM(pin_motor_forward, 50)
motor_backward = GPIO.PWM(pin_motor_backward, 50)
steering_motor = GPIO.PWM(pin_steering_motor, 50)


motor_forward.start(0)
motor_backward.start(0)
steering_motor.start(0)








