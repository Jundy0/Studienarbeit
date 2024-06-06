##########      this file collects all sensordata       ##########



import time
import data_setup
import output_setup
import input_setup
import RPi.GPIO as GPIO


# if this function is called, a list of all 4 sensor-values is delivered back
def distance():
    #for i in range(0,3):

        cnt_helper = 0
        GPIO.output(output_setup.trigger_pins_us_sensors[data_setup.number_us_sensor], True)

        # wait 0.00001 before deactivate the trigger output
        time.sleep(0.00001)
        GPIO.output(output_setup.trigger_pins_us_sensors[data_setup.number_us_sensor], False)

        starttime = time.time()
        stoptime = time.time()

        # safe the starttime
        while GPIO.input(input_setup.echo_pins_us_sensors[data_setup.number_us_sensor]) == 0:
            starttime = time.time()
            cnt_helper = cnt_helper +1
            #print("gtgggg")
            if cnt_helper > 50:
                return

        # safe the time when the echo is back
        while GPIO.input(input_setup.echo_pins_us_sensors[data_setup.number_us_sensor]) == 1:
            stoptime = time.time()

            

        # calculate the time difference from emission to receive
        diff_time = stoptime - starttime

        # do some cool mathe stuff to calculate the distance
        # multiply the time with the speed of sound and divide it by two to get the distance from the sensor to the reflection
        distance = (diff_time * 34300) / 2
        
        if data_setup.number_us_sensor == 0:
            data_setup.distance_us_left_front = distance
            print("links vorne:"+str(data_setup.distance_us_left_front))
        elif data_setup.number_us_sensor == 1:
            data_setup.distance_us_mid_front = distance
            print("mitte vorne:"+str(data_setup.distance_us_mid_front))
        elif data_setup.number_us_sensor == 2:
            data_setup.distance_us_right_front = distance
            print("rechts vorne:"+str(data_setup.distance_us_right_front))
        elif data_setup.number_us_sensor == 3:
            data_setup.distance_us_mid_back = distance
            print("hinten:"+str(data_setup.distance_us_mid_back))
            
        #data_setup.number_us_sensor = data_setup.number_us_sensor+1
        #if data_setup.number_us_sensor == 2:
            #data_setup.number_us_sensor = 3
        #if data_setup.number_us_sensor == 4:
            #data_setup.number_us_sensor = 1
        return
        
        
        
        
        


