##########      in this file, the Lidar data will be used for calculating a movement without a crash      ##########

import random
import data_setup
needed_values = 30
mid_value_list = 0

import data_setup

lidar_data = [None] * 120

for i in range(0, 120):
   lidar_data[i] = random.randint(0,300)

if data_setup.input_data[data_setup.speed_forward] > 0:
    print("1")
    sum_1 = 0
    sum_2 = 0
    mid_value_list =  30 + data_setup.input_data[data_setup.steering_angle] * needed_values
    for i in range(mid_value_list-15, mid_value_list):
        sum_1 = sum_1 + lidar_data[i]
        average_1 = sum_1 / (needed_values/2)

    for i in range(mid_value_list, mid_value_list + 15):
        sum_2 = sum_2 + lidar_data[i]
        average_2 = sum_2 / (needed_values / 2)


    if average_1 < average_2:
        print("eher rechts fahren bitte!")
    else:
        print("eher links fahren bitte!")



elif data_setup.input_data[data_setup.speed_backward] >= 0:
    print("2")
    sum_1 = 0
    sum_2 = 0
    mid_value_list =  90 - data_setup.input_data[data_setup.steering_angle] * needed_values
    for i in range(mid_value_list-15, mid_value_list):
        sum_1 = sum_1 + lidar_data[i]
        average_1 = sum_1 / (needed_values/2)

    for i in range(mid_value_list, mid_value_list + 15):
        sum_2 = sum_2 + lidar_data[i]
        average_2 = sum_2 / (needed_values / 2)


    if average_1 < average_2:
        print("eher links fahren bitte!")
        GPIO.output(output_setup.pin_left_LED, True)
        GPIO.output(output_setup.pin_right_LED, False)

    else:
        print("eher rechts fahren bitte!")
        GPIO.output(output_setup.pin_left_LED, False)
        GPIO.output(output_setup.pin_right_LED, True)