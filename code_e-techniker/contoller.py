##########      in this file, magic about the controller is happening      ##########
import data_setup
import pygame
#initialize Pygame and the controller
pygame.joystick.init()
pygame.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
print("Your Controller: ", joysticks[0])



#initialize all pygame stuff (not needed at the moment)
#pygame.time.Clock()
#pygame.display.set_mode((800, 600))

# get all the needed controller data to move and control the vehicle
def get_data():

    '''
        Controller Data:        Range:         Datatype:
            Speed_forward       0...1           float
            speed_backward      0...1           float
            steering_angle      -1...1          float
            button_A           True/False       boolean
            button_B           True/False       boolean
            button_X           True/False       boolean
            button_Y           True/False       boolean
            button_RL          True/False       boolean
            button_RB          True/False       boolean
            button_back        True/False       boolean
            button_start       True/False       boolean

    '''

    #default all the values back to non controlled status
    right_shoulder_RT = 0
    left_shoulder_LT = 0
    right_ball = 0
    button_A = False
    button_B = False
    button_X = False
    button_Y = False
    button_LB = False
    button_RB = False
    button_back = False
    button_start = False



    for event in pygame.event.get():
        #if statements is True when ether one of the lower shoulder button or the right ball is moved
        if event.type == pygame.JOYAXISMOTION:
            right_shoulder_RT= round((pygame.joystick.Joystick(0).get_axis(5) + 1)/2, 2)    #Quick math to turn the value from the range -1...1 to a range 0...1
            left_shoulder_LT = round((pygame.joystick.Joystick(0).get_axis(2) + 1)/2, 2)    #Quick math to turn the value from the range -1...1 to a range 0...1
            right_ball = round(pygame.joystick.Joystick(0).get_axis(3), 2)           #Range from ball is -1...1, this range is needed to decide in which direction the ball is moved
            

        #if statement is True when on of the pressable buttons get pressed down, exept for the cross button on the left
        if event.type == pygame.JOYBUTTONDOWN:
            button_A = pygame.joystick.Joystick(0).get_button(0)    # Green Button A
            button_B = pygame.joystick.Joystick(0).get_button(1)    # Red Button B
            button_X = pygame.joystick.Joystick(0).get_button(2)    # Blue Button X
            button_Y = pygame.joystick.Joystick(0).get_button(3)    # Yellow Button Y
            button_LB = pygame.joystick.Joystick(0).get_button(4)    # Upper shoulder left Button LB
            button_RB = pygame.joystick.Joystick(0).get_button(5)    # Upper shoulder right Button RB
            button_back = pygame.joystick.Joystick(0).get_button(6)     # start button in center of the controller
            button_start = pygame.joystick.Joystick(0).get_button(7)    # back button in center of the controller


       

    data_setup.input_data = [right_shoulder_RT, left_shoulder_LT, right_ball, button_A, button_B, button_X, button_Y, button_LB, button_RB, button_back, button_start]
    #return [right_shoulder_RT, left_shoulder_LT, right_ball, button_A, button_B, button_X, button_Y, button_LB, button_RB, button_back, button_start]
    #print(data_setup.input_data)



