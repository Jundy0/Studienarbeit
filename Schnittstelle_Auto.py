# Controller Data:        Range:         Datatype:
#   Speed_forward         0...1           float
#   speed_backward        0...1           float
#   steering_angle        -1...1          float
#   button_A             True/False       boolean
#   button_B             True/False       boolean
#   button_X             True/False       boolean
#   button_Y             True/False       boolean
#   button_RL            True/False       boolean
#   button_RB            True/False       boolean

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


def vehicle_tasks(command_list):

# DC motor controlling to move the vehicle for- and backwards
    if command_list[data_setup.speed_forward] != 0 and command_list[data_setup.speed_backward] == 0: #only the button to drive forward is presses
        #PWM to set backward speed to Zero
        ##!output_setup.motor_backward.ChangeDutyCycle(0)
        if  command_list[data_setup.speed_forward] >-0.05 and command_list[data_setup.speed_forward] <0.05: #delete some hardware gitter of the Controller
            command_list[data_setup.speed_forward] = 0
            #PWM to set forward speed to Zero
            ##!output_setup.motor_forward.ChangeDutyCycle(0)
            print("forward velocity = 0!\n")
        else:
            print("vehicle moves with " + str(command_list[data_setup.speed_forward]*100)+"% of maximum velocity(forward).\n")
            # !!!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!
            ##!output_setup.motor_forward.ChangeDutyCycle(command_list[data_setup.speed_forward] * 100)

    elif command_list[data_setup.speed_backward] != 0 and command_list[data_setup.speed_forward] == 0: #only the button to drive backward is presses
        # PWM to set forward speed to Zero
        ##!output_setup.motor_forward.ChangeDutyCycle(0)
        if  command_list[data_setup.speed_backward] >-0.05 and command_list[data_setup.speed_backward] <0.05: #delete some hardware gitter of the Controller
            command_list[data_setup.speed_backward] = 0
            ##!output_setup.motor_backward.ChangeDutyCycle(0)
            # PWM to set backward speed to Zero
            print("backward velocity = 0!\n")
        else:
            print("vehicle moves with " + str(command_list[data_setup.speed_backward]*100)+"% of maximum velocity(backward).\n")
            # !!!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!
            ##!output_setup.motor_backward.ChangeDutyCycle(command_list[data_setup.speed_backward]*100)
    elif command_list[data_setup.speed_forward] != 0 and command_list[data_setup.speed_backward] != 0: #both buttons to move for- and backwards are pressed
        # PWM to set forward speed to Zero
        ##!output_setup.motor_forward.ChangeDutyCycle(0)
        print("forward velocity = 0!\n")
        ##!output_setup.motor_backward.ChangeDutyCycle(0)
        # PWM to set backward speed to Zero
        print("backward velocity = 0!\n")



# Servomotor controlling to steer the front axis
    if command_list[data_setup.steering_angle] != 0:
        if command_list[data_setup.steering_angle] > -0.05 and command_list[data_setup.steering_angle] < 0.05:
            command_list[data_setup.steering_angle] = 0
            servo_controller.move_servo(command_list[data_setup.steering_angle])
            print("steering Angle = 0!\n")
        else:
            if command_list[data_setup.steering_angle] < 0:  # steering angle is negative, vehicle should drive to the left
                print("vehicle steering with " + str(command_list[data_setup.steering_angle] * 100) + "% of maximum angle (left).\n")
                # !!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!!
                servo_controller.move_servo(command_list[data_setup.steering_angle])
            else:  # steering angle is positive, vehicle should drive to the right
                print("vehicle moves with " + str(command_list[data_setup.steering_angle] * 100) + "% of maximum angle (right).\n")
                # !!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!!
                servo_controller.move_servo(command_list[data_setup.steering_angle])