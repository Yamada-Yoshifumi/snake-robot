"""snake_basic controller."""

#********* IMPORTS *********#
# Import basic py libs
import cv2 as cv2   # opencv
import numpy as np

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# from controller import Motor


#********* START ROBOT *********#
"""
Creates instance of robot, initialises timestep of simulation (note this is NOT the same as timestep for camera)
"""

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())       # in milliseconds
print(f"simulation timestep = {timestep} ms")





#********* INITIALISATION *********#

# !!!!!!!! TURN SNAKE MODE ON OR OFF !!!!!!!! #
# True to keep legs, False to extend legs
snake_mode = False   
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! #

# Initial locomotion values
spine_offset = 0.0  # this controls turning. negative vals to turn left, positive turns right
amplitude = 1.0     # Gait eqn amplitude
phase = 0.0         # current locomotion phase
walk_ampl = 0.6     # Walk amplitude in rads (for body swaying)
freq = 1.4          # Gait freq in Hz

# Motor position (rads) to keep legs
KEEP_LEGS_RAD = -2.0




#********* GLOBAL CONSTANTS *********#

# Number of motors: 6 actuated body segments and 4 legs
NUM_MOTORS = 10     

# Initialise list to store target motor positions
target_position = np.zeros(NUM_MOTORS)




#********* START CAMERA *********#
"""
Command to start camera and get image data from it
"""
## image capture rate of camera
CAMERA_RATE = timestep      # this is default timestep (32 ms), so camera will refresh at 31.25Hz
# CAMERA_RATE = 2500          # this is the slower setting as per pape;, one image per 2.5s (0.4 Hz)
print(f"camera takes one pic every {CAMERA_RATE} ms")

# Start camera
camera = robot.getDevice('camera')
camera.enable(CAMERA_RATE)
img_width = camera.getWidth()
img_height = camera.getHeight()
print(f"dims of img: w={img_width}, h={img_height}")




#********* START MOTORS *********#
"""
Commands to start motors and initialise their initial positions and velocity
"""

# Get Names of motors as per webots
MOTOR_NAMES = ["motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4"]

# Store motors as a list
motors = [robot.getDevice(MOTOR_NAMES[i]) for i in range(NUM_MOTORS)]

# Set min/max position of motors
min_motor_positions = [motors[i].getMinPosition() for i in range(NUM_MOTORS)]
max_motor_positions = [motors[i].getMaxPosition() for i in range(NUM_MOTORS)]
print(f"min motor pos = {min_motor_positions}") # Debug
print(f"max motor pos = {max_motor_positions}") # Debug


# Set initial positions of motor
for i in range(NUM_MOTORS):
    motors[i].setPosition(0)
    # motors[i].setVelocity(float('inf'))




#********* HELPER FUNCTIONS *********#
"""
Put helper functions here. E.g., Gait equation, path selection, and obstacle avoidance
"""

def restrict(target_pos, min_pos, max_pos):
    """
    Clamps motor position to within min/max values 
    """
    if (min_pos == 0 and max_pos == 0):     # weird case, just remain unchanged
        return target_pos
    else:   # clamps value to limit
        return max(min_pos, min(max_pos, target_pos))


def get_image():
    """
    Gets image and convert into cv2 readable format

    Reference: https://github.com/lukicdarkoo/webots-example-visual-tracking
    """
    # Get image after 1 step
    # robot.step(timestep)          # UNCOMMENT IF IMG TYPE IS NONE
    # cameraData = camera.getImage()
    cameraData = camera.getImageArray()

    # Process image into numpy array
    # img = np.frombuffer(cameraData, np.uint8).reshape((img_height, img_width, 4))
    # print(img)
    img = np.asarray(cameraData, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)

    
    # print(f"cv ver = {cv2.__version__ }")     # Debug
    img_cv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    img_cv = cv2.rotate(img_cv, cv2.ROTATE_90_CLOCKWISE)
    img_cv = cv2.flip(img_cv, 1)

    return img_cv



#********* MAIN CONTROL LOOP *********#
"""
Put all actuation commands here. Robot basically runs this while loop and exits the loop when u press stop in Webots
"""

while robot.step(timestep) != -1:

    ## ***** 1. Read the sensors ***** ##

    # Get an image from the camera and convert to OpenCV format
    img_cv = get_image()



    ## ***** 2. Calculate output actuator commands here ***** ##

    # Make robot go straight. But if you want it to turn, then adjust accordingly
    spine_offset = 0.0

    # Increase phase according to elapsed time
    phase -= (timestep / 1000) * freq * 2 * np.pi

    # Constants that enable S-shaped body of robot (per demo)
    A = [-0.7, 1, 1, 0, -1, -1]

    # Calculate motor positions for body swaying
    for i in range(6):  # up to 6 because motors 0-5 for body. 6-9 for leg
        target_position[i] = walk_ampl * amplitude * A[i] * np.sin(phase) + spine_offset
        # The above is a sample eqn for body wave. Replace it with gait eqn as needed!!!


    # Calculate leg movements
    if snake_mode:      # in snake mode, so legs are kept
        for i in range(6, NUM_MOTORS):
            target_position[i] = KEEP_LEGS_RAD

    else:   # Not in snake mode, so rotate legs as robot walk
        target_position[6] = phase
        target_position[7] = phase + np.pi
        target_position[8] = phase + np.pi
        target_position[9] = phase



    ## ***** 3. Set motor actuation ***** ##
    for i in range(NUM_MOTORS):

        # Ensure does not exceed min/max of motor
        target_position[i] = restrict(target_position[i], min_motor_positions[i], max_motor_positions[i])

        # Debug
        print(f"final target pos = {target_position}")

        # Finally, set motor position
        # motors[i].setVelocity(target_position[i])
        motors[i].setPosition(target_position[i])
    


# Enter here exit cleanup code.
