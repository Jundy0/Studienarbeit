##########      in this file, all needed values get defined      ##########

# loop values to slow down processor
number_of_iterations = 1000
running_variable = 0

helper_us_sensor_delay = 0 # counter for waiting next us-data generating
number_us_sensor = 1

#counters how often a function is called


#list indexes of the input_data list
speed_forward = 0
speed_backward = 1
steering_angle = 2
button_A = 3
button_B = 4
button_X = 5
button_Y = 6
button_RL = 7
button_RB = 8
button_back = 9
button_start = 10
input_data = [0,0,0,0,0,0,0,0,0,0,0]

# driving modes
driving_mode = 0

# values US Sensors
distance_us_left_front = 0
distance_us_mid_front = 0
distance_us_right_front = 0
distance_us_mid_back = 0

safety_offset = 10 # offset is needed to calculate a safe-distance from at least 10cm before the vehicle stops


