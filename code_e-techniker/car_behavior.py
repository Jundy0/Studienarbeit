##########      in this file, the inputs will control the actuators      ##########

import data_setup
import output_setup
import servo_controller
import sensor_data

def moving_management():
    if data_setup.driving_mode == 1:
        vehicle_tasks()
        
    elif data_setup.driving_mode == 2:
       # get the needed sensor values

        
        if data_setup.helper_us_sensor_delay > 10:
            sensor_data.distance()
            data_setup.helper_us_sensor_delay = 0
        else:
            data_setup.helper_us_sensor_delay = data_setup.helper_us_sensor_delay + 1
        
        # calculate the forward speed dependent on the current speed and the distance
        #collision_detect()

        vehicle_tasks()

    elif data_setup.driving_mode == 3:
        #tbd this mode contains the algorithmic driving
        #!!!!!!!!! data_setup.input_data = DATA FROM INFORMATIKTEAM !!!!!!!!!!!!!
        vehicle_tasks(data_setup.input_data)





def collision_detect():
    #print("links vorne: ",data_setup.distance_us_left_front)
    print("mitte vorne: ",data_setup.distance_us_mid_front)
    #print("rechts vorne: ",data_setup.distance_us_right_front)
    print("mitte hinten: ",data_setup.distance_us_mid_back)
    print("_______________________________________________________________")
    
    
    return




def vehicle_tasks():
    


# DC motor controlling to move the vehicle for- and backwards
    if data_setup.input_data[data_setup.speed_forward] != 0 and data_setup.input_data[data_setup.speed_backward] == 0: #only the button to drive forward is presses
        #PWM to set backward speed to Zero
        output_setup.motor_backward.ChangeDutyCycle(0)
        if  data_setup.input_data[data_setup.speed_forward] >-0.05 and data_setup.input_data[data_setup.speed_forward] <0.05: #delete some hardware gitter of the Controller
            data_setup.input_data[data_setup.speed_forward] = 0
            #PWM to set forward speed to Zero
            output_setup.motor_forward.ChangeDutyCycle(0)
            #print("forward velocity = 0!\n")
        else:
            #print("vehicle moves with " + str(data_setup.input_data[data_setup.speed_forward]*100)+"% of maximum velocity(forward).\n")
            # !!!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!
            output_setup.motor_forward.ChangeDutyCycle(data_setup.input_data[data_setup.speed_forward] * 100)

    elif data_setup.input_data[data_setup.speed_backward] != 0 and data_setup.input_data[data_setup.speed_forward] == 0: #only the button to drive backward is presses
        # PWM to set forward speed to Zero
        output_setup.motor_forward.ChangeDutyCycle(0)
        if  data_setup.input_data[data_setup.speed_backward] >-0.05 and data_setup.input_data[data_setup.speed_backward] <0.05: #delete some hardware gitter of the Controller
            data_setup.input_data[data_setup.speed_backward] = 0
            output_setup.motor_backward.ChangeDutyCycle(0)
            # PWM to set backward speed to Zero
            #print("backward velocity = 0!\n")
        else:
            #print("vehicle moves with " + str(data_setup.input_data[data_setup.speed_backward]*100)+"% of maximum velocity(backward).\n")
            # !!!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!
            output_setup.motor_backward.ChangeDutyCycle(data_setup.input_data[data_setup.speed_backward]*100)
    elif data_setup.input_data[data_setup.speed_forward] != 0 and data_setup.input_data[data_setup.speed_backward] != 0: #both buttons to move for- and backwards are pressed
        # PWM to set forward speed to Zero
        output_setup.motor_forward.ChangeDutyCycle(0)
        #print("forward velocity = 0!\n")
        output_setup.motor_backward.ChangeDutyCycle(0)
        # PWM to set backward speed to Zero
        #print("backward velocity = 0!\n")



# Servomotor controlling to steer the front axis
    if data_setup.input_data[data_setup.steering_angle] != 0:
        if data_setup.input_data[data_setup.steering_angle] > -0.05 and data_setup.input_data[data_setup.steering_angle] < 0.05:
            data_setup.input_data[data_setup.steering_angle] = 0
            servo_controller.move_servo(data_setup.input_data[data_setup.steering_angle])
            #print("steering Angle = 0!\n")
        else:
            if data_setup.input_data[data_setup.steering_angle] < 0:  # steering angle is negative, vehicle should drive to the left
                #print("vehicle steering with " + str(data_setup.input_data[data_setup.steering_angle] * 100) + "% of maximum angle (left).\n")
                # !!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!!
                servo_controller.move_servo(data_setup.input_data[data_setup.steering_angle])
            else:  # steering angle is positive, vehicle should drive to the right
                #print("vehicle moves with " + str(data_setup.input_data[data_setup.steering_angle] * 100) + "% of maximum angle (right).\n")
                # !!!!!!!!!add PWM stuff to speed up and slow down the DC motor!!!!!!!!!!
                servo_controller.move_servo(data_setup.input_data[data_setup.steering_angle])

# next section controlles anything else like future light and so on....


