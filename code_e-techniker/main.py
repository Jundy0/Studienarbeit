##########      main file      ##########
'''
    input_data is a list, this list contains all inputs from the controller.
    Element     value
    0           speed_forward
    1           speed_backward
    2           steering_angle
    3           button_A (tdb)
    4           button_B (tdb)
    5           button_X (tdb)
    6           button_Y (tdb)
    7           button_RL (tdb)
    8           button_RB (tdb)
    9           button_back
    10          button_start

'''

import RPi.GPIO as GPIO
import contoller
import data_setup
import car_behavior
import output_setup

start_helper = False


while True:
    contoller.get_data()


    # driving mode administration, you can change the mode only with a none moving vehicle
    if data_setup.input_data[data_setup.button_RB] == True and data_setup.driving_mode == 0:
        data_setup.driving_mode = 1 # in this mode, the vehicle move with the given controller inputs
        print("mode= :", data_setup.driving_mode,"controller without sensors")
        #GPIO.output(output_setup.pin_mode_LED_1, True)
        #GPIO.output(output_setup.pin_mode_LED_2, False)

    elif data_setup.input_data[data_setup.button_RB] == True and data_setup.driving_mode == 1:
        data_setup.driving_mode = 2 # in this mode, the vehicle moves with the give inputs, collision avoiding with US-Sensors
        print("mode= :", data_setup.driving_mode, "controller wit US sensors")
        #GPIO.output(output_setup.pin_mode_LED_1, False)
        #GPIO.output(output_setup.pin_mode_LED_2, True)

    elif data_setup.input_data[data_setup.button_RB] == True and data_setup.driving_mode == 2:
        data_setup.driving_mode = 3 # in this mode, the vehicle moves with the algorithm from the other team
        print("mode= :", data_setup.driving_mode, "Lidarmode")
        #GPIO.output(output_setup.pin_mode_LED_1, True)
        #GPIO.output(output_setup.pin_mode_LED_2, True)

    elif data_setup.input_data[data_setup.button_RB] == True and data_setup.driving_mode == 3:
        data_setup.driving_mode = 1
        print("mode= :", data_setup.driving_mode)

    if data_setup.input_data[data_setup.button_start] == True:
        start_helper = True



    while start_helper == True:
        input_list = contoller.get_data()

        if  data_setup.input_data[data_setup.button_back] == True:
            start_helper = False

        car_behavior.moving_management()
