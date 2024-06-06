
import output_setup

#values from Datasheet, puls range from 500us to 2500us neutral position at 1500us
frequency = 50
neutral_duty_cycle = 7.5
minimum_duty_cycle = 5
maximus_duty_cycle = 10

def move_servo(movement_value):

    duty_cycle = neutral_duty_cycle + (movement_value * (neutral_duty_cycle-minimum_duty_cycle))
    if duty_cycle <= maximus_duty_cycle and duty_cycle >= minimum_duty_cycle:
        output_setup.steering_motor.ChangeDutyCycle(duty_cycle)
        print("DutyCycle in range: "+ str(duty_cycle))
    else:
        duty_cycle = neutral_duty_cycle
        output_setup.steering_motor.ChangeDutyCycle(neutral_duty_cycle)
        print("DutyCycle in range: "+ str(duty_cycle))