"""snake_basic controller."""

#********* IMPORTS *********#
# Import basic py libs
import cv2 as cv2   # opencv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from controller import Robot
# from controller import Motor


# Motor position (rads) to keep legs
KEEP_LEGS_RAD = -2.0

#********* GLOBAL CONSTANTS *********#

# Number of motors: 6 actuated body segments and 4 legs
NUM_MOTORS = 10  
# Get Names of motors as per webots
MOTOR_NAMES = ["motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4"]

## image capture rate of camera in ms
# Write "default" to use default rate of 32 ms, but here we use slower rates as per paper
CAMERA_RATE = 500   # paper used 2500ms
# CAMERA_RATE = 'default'   

#********* HELPER FUNCTIONS *********#
"""
Put helper functions here. E.g., Gait equation, path selection, and obstacle avoidance
"""
class SnakeRobotController:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())       # in milliseconds
        # Record when the previous image was taken
        self.previous_sample_time = int(self.robot.getTime())    # in seconds

        rclpy.init(args=None)
        self.node = rclpy.create_node('snake_basic')
        self.pub = self.node.create_publisher( Float32MultiArray, 'motor', 10)
        self.node.get_logger().info(f"simulation timestep = {self.timestep} ms")    # DEBUG

        #********* INITIALISATION *********#

        # !!!!!!!! TURN SNAKE MODE ON OR OFF !!!!!!!! #
        # True to keep legs, False to extend legs
        self.snake_mode = False   
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! #

        # Initial locomotion values
        self.spine_offset = 0.0  # this controls turning. negative vals to turn left, positive turns right
        self.amplitude = 1.0     # Gait eqn amplitude
        self.phase = 0.0         # current locomotion phase
        self.walk_ampl = 0.6     # Walk amplitude in rads (for body swaying)
        self.freq = 1.0         # Gait freq in Hz   # Changed from 1.4 to 1 to make camera_rate an int

        # Initialise list to store target motor positions
        self.target_position = np.zeros(NUM_MOTORS)

        #********* START CAMERA *********#
        """
        Command to start camera and get image data from it
        """
        
        # Start camera
        self.camera = self.robot.getDevice('camera')

        # Select camera refresh rate
        cam_rate = int(1000 / self.freq)    # in miliseconds
        # if isinstance(CAMERA_RATE, int):
        #     cam_rate = CAMERA_RATE
        # else:
        #     self.node.get_logger().info("No custom refresh rate specified, reverting to default value.")
        #     cam_rate = self.timestep # this is default timestep (32 ms), so camera will refresh at 31.25Hz

        self.node.get_logger().info(f"camera takes one pic every {cam_rate} ms")

        self.camera.enable(cam_rate)
        self.img_width = self.camera.getWidth()
        self.img_height = self.camera.getHeight()
        self.node.get_logger().info(f"dims of img: w={self.img_width}, h={self.img_height}")
        #********* START MOTORS *********#
        """
        Commands to start motors and initialise their initial positions and velocity
        """

        # Store motors as a list
        self.motors = [self.robot.getDevice(MOTOR_NAMES[i]) for i in range(NUM_MOTORS)]

        # Set min/max position of motors
        self.min_motor_positions = [self.motors[i].getMinPosition() for i in range(NUM_MOTORS)]
        self.max_motor_positions = [self.motors[i].getMaxPosition() for i in range(NUM_MOTORS)]
        self.node.get_logger().info(f"min motor pos = {self.min_motor_positions}") # Debug
        self.node.get_logger().info(f"max motor pos = {self.max_motor_positions}") # Debug
        # Set initial positions of motor
        for i in range(NUM_MOTORS):
            self.motors[i].setPosition(0)
            # motors[i].setVelocity(float('inf'))

    def restrict(self, target_pos, min_pos, max_pos):
        """
        Clamps motor position to within min/max values 
        """
        if (min_pos == 0 and max_pos == 0):     # weird case, just remain unchanged
            return target_pos
        else:   # clamps value to limit
            return max(min_pos, min(max_pos, target_pos))


    def get_image(self):
        """
        Gets image and convert into cv2 readable format

        Reference: https://github.com/lukicdarkoo/webots-example-visual-tracking
        """
        # Get image after 1 step
        #self.robot.step(self.timestep)          # UNCOMMENT IF IMG TYPE IS NONE
        #cameraData = self.camera.getImage()
        cameraData = self.camera.getImageArray()

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
    
    def segmentation(self, img):
        """
        Segement the image into ground pixels and non-ground pixels.
        Multiple methods will be implemented:
        1. Naive thresholding (done)
        2. SVM (TODO)
        3. Others (TODO)
        """
        h, w, _ = img.shape
        mid_w = int(np.floor(w / 2))
        w_low = mid_w - int(np.floor(w / 10))
        w_high = mid_w + int(np.floor(w / 10))
        h_low = h - int(np.floor(h / 10))
        ground_ref_img = img[h_low:h, w_low:w_high, :]    # 24x8 image, can be tuned later
        # Debugging!
        # print(ground_ref_img.shape)
        # img_RGB = cv2.cvtColor(ground_ref_img, cv2.COLOR_HSV2RGB)
        # cv2.namedWindow("Ground reference image")
        # cv2.imshow("Ground reference image", img_RGB)
        # cv2.waitKey(0)
        # print(ground_ref_img)
        tol = np.array([10, 30, 0])
        low_H, low_S, _ = np.min(ground_ref_img, axis=(0,1)) - tol
        high_H, high_S, _ = np.max(ground_ref_img, axis=(0,1)) + tol
        # self.node.get_logger().info(f"{low_H}, {low_S}, {20}, {high_H}, {high_S}, {255}")
        mask = cv2.inRange(img, np.array([low_H, low_S, 20]), np.array([high_H, high_S, 255]))
        # cv2.namedWindow("Mask")
        # cv2.imshow("Mask", mask)
        # cv2.waitKey(0)
        return mask


    def trajectory_sampling(self, h, w):
        """
        Sample 5 trajectories (for now only 1) that will be tested,
        return the end points (can be modified to return a whole trajectory).
        """
        mid_w = int(np.floor(w / 2))
        mid_h = int(np.floor(h / 2)) #+ int(np.floor(h / 20))  # Add a tolerance because of head's vertical tilting

        candidate_points = []
        # Straight forward
        candidate_points.append(np.array([mid_h, mid_w]))
        # Left turn: spine_offset=-0.3
        candidate_points.append(np.array([mid_h, 7]))  # Manually calculated
        # Right turn: spine_offset=0.3
        candidate_points.append(np.array([mid_h, w-7-1]))  # To account for array index

        # Debugging!
        # self.node.get_logger().info(f"Candidate points: {candidate_points}")
        return candidate_points


    def collision_check(self, mask, points, h, w, vertical_length=False):
        """
        Calculate how long can each trajectory (represented by the end points) can travel
        in image space before collision.
        
        :param Boolean vertical_length: True for vertical length in image space
        :return the index of the trajectory that can travel the longest length.
        """

        # NOTE: The following is outdated code being commented out.
        # It test a small patch around the end-point and use that to determine whether
        # a trajectory is collision-free.
        # This is no longer used since we are calculating the length of trajectories.

        # row = point[0]
        # column = point[1]

        # # If not enough space around the point, just check the single point.
        # # This shall not happen for the default setting (120x80 image), so a simple solution is chosen.
        # # A better solution would be to do extrapolation at borders, for example.
        # if row - 1 < 0 or row + 1 >= h or column - 1 < 0 or column + 1 >= w:
        #     return mask[row, column]
        
        # # Otherwise, each pixel in the 3x3 patch casts a vote.
        # score = np.sum(mask[row-1:row+2, column-1:column+2]) / 255 / 9
        # # Debugging!
        # # self.node.get_logger().info(f"Patch: {mask[row-1:row+2, column-1:column+2]}")
        # # self.node.get_logger().info(f"Score: {score}")
        # # Trajectory is deemed collision-free if at least 60% of the pixels are ground pixels
        # return score > 0.6

        # New code starts from here
        mid_w = int(np.floor(w / 2))
        mid_h = int(np.floor(h / 2))
        row_start = h - 1       # Starting point
        column_start = mid_w
        length = np.array([0.0, 0.0, 0.0])        # Record length of trajectories
        end_flag = [False, False, False]    # Record whether a trajecory has encountered an obstacle
        step1 = (points[1][1] - mid_w) / (h - mid_h)
        step2 = (points[2][1] - mid_w) / (h - mid_h)

        if vertical_length:
            length_per_row = 1
        else:
            # Calculate the actual length (approximate by straignt lines) for the turning options (assume symmetric),
            # divide by the number of rows, so this is the increment for each row
            length_per_row = np.sqrt((points[1][1] - mid_w)**2 + (h - mid_h)**2) / (h - mid_h)

        # Starting from the central bottom pixel, track the trajectories by moving one row up at a time.
        # The projections of trajectories are simplified to straight lines.
        for i in range(h - mid_h):
            # End early if all trajectories have collisions
            if end_flag[0] and end_flag[1] and end_flag[2]:
                break
            
            # Decrement row
            row = row_start - i

            # Check first trajectory (straight forward)
            if not end_flag[0]:
                column = column_start
                if mask[row, column]:
                    length[0] += 1
                else:
                    end_flag[0] = True
            # Check second trajectory (turning left)
            if not end_flag[1]:
                column = int(np.floor(column_start + step1*i))
                if mask[row, column]:
                    length[1] += length_per_row
                else:
                    end_flag[1] = True
            # Check third trajectory (turning right)
            if not end_flag[2]:
                column = int(np.floor(column_start + step2*i))
                if mask[row, column]:
                    length[2] += length_per_row
                else:
                    end_flag[2] = True

        # Return the number of the trajectory with the longest length
        self.node.get_logger().info(f"Length: {length}")
        max_length = np.max(length)
        if max_length <= h/4:       # If obstacle is already too close,
            choice = 3              # take the sharp left turn
        else:
            choice = np.argmax(length)
        self.node.get_logger().info(f"Choice: {choice}")
        return choice


    def get_direction(self, img):
        """
        Given the input image, output the spine_offset control which steers the salamander.
        """
        h, w, _ = img.shape

        # Get the ground segmentation mask
        mask = self.segmentation(img)

        # Sample some trajectories, return the end points
        candidate_points = self.trajectory_sampling(h, w)

        # Collision checking for the candidate trajectories
        # Choose a trajectory from the following four options:
        # 0. Go straight forward
        # 1. Turn left by setting spine_offset = -0.3
        # 2. Turn left by setting spine_offset = 0.3
        # 3. Turn left by setting spine_offset = -0.6
        # The last case is for emergency collision avoidance. This causes the salamander to 
        # take a sharp turn and its new location is out of the viewing frustrum at the old location,
        # so no need to (or actually, we cannot) test for that case.
        choice = self.collision_check(mask, candidate_points, h, w)

        # Debugging!
        # img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        # cv2.namedWindow("Input image")
        # cv2.imshow("Input image", img_RGB)
        # cv2.waitKey(0)
        # cv2.namedWindow("Mask")
        # cv2.imshow("Mask", mask)
        # cv2.waitKey(0)

        # Save some images for illustration
        # if choice == 2:
        #     img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        #     cv2.imwrite("/home/kaicao/right_turn_image.png", img_RGB)
        #     self.node.get_logger().info(f"Image saved!")
        #     cv2.imwrite("/home/kaicao/right_turn_mask.png", mask)
        #     self.node.get_logger().info(f"Mask saved!")
        # if choice == 1:
        #     img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        #     cv2.imwrite("/home/kaicao/left_turn_image.png", img_RGB)
        #     self.node.get_logger().info(f"Image saved!")
        #     cv2.imwrite("/home/kaicao/left_turn_mask.png", mask)
        #     self.node.get_logger().info(f"Mask saved!")
        # if choice == 0:
        #     img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        #     cv2.imwrite("/home/kaicao/forward_image.png", img_RGB)
        #     self.node.get_logger().info(f"Image saved!")
        #     cv2.imwrite("/home/kaicao/forward_mask.png", mask)
        #     self.node.get_logger().info(f"Mask saved!")

        if  choice == 0:
            return 0.0
        elif choice == 1:
            return -0.3
        elif choice == 2:
            return 0.3
        else:
            return -0.6

        # Should never reach here
        return 0.0

    def step(self):

        ## ***** 1. Read the sensors ***** ##

        # Get an image from the camera and convert to OpenCV format
        img_cv = self.get_image()

        ## ***** 2. Calculate output actuator commands here ***** ##

        # (outdated)Make robot go straight. But if you want it to turn, then adjust accordingly
        if img_cv is not None:
            time = int(self.robot.getTime())
            cam_rate = int(1 / self.freq)    # This time in seconds!
            if time - self.previous_sample_time >= cam_rate:

                self.previous_sample_time = time
                # Get the direction using trajectory sampling and collision checking in perception space
                self.spine_offset = self.get_direction(img_cv)
                # self.spine_offset = -0.3  # Constant setting only used for testing image sampling!
                self.node.get_logger().info(f"Setting spine offset: {self.spine_offset}")
                
                # NOTE: The following is only used for testing image sampling
                # This requires to set the Robot as a Supervisor and set its DEF as "salamander" in the wbt file
                # salamander_robot = self.robot.getFromDef("salamander")
                # self.node.get_logger().info(f"Robot position: {salamander_robot.getPosition()}")
                # self.node.get_logger().info(f"Robot orientation: {salamander_robot.getOrientation()}")

        # Increase phase according to elapsed time
        self.phase -= (self.timestep / 1000) * self.freq * 2 * np.pi

        # Constants that enable S-shaped body of robot (per demo)
        A = [-0.7, 1, 1, 0, -1, -1]

        # Calculate motor positions for body swaying
        for i in range(6):  # up to 6 because motors 0-5 for body. 6-9 for leg
            self.target_position[i] = self.walk_ampl * self.amplitude * A[i] * np.sin(self.phase) + self.spine_offset
            # The above is a sample eqn for body wave. Replace it with gait eqn as needed!!!


        # Calculate leg movements
        if self.snake_mode:      # in snake mode, so legs are kept
            for i in range(6, NUM_MOTORS):
                self.target_position[i] = KEEP_LEGS_RAD

        else:   # Not in snake mode, so rotate legs as robot walk
            self.target_position[6] = self.phase
            self.target_position[7] = self.phase + np.pi
            self.target_position[8] = self.phase + np.pi
            self.target_position[9] = self.phase

        ## ***** 3. Set motor actuation ***** ##
        for i in range(NUM_MOTORS):

            # Ensure does not exceed min/max of motor
            self.target_position[i] = self.restrict(self.target_position[i], self.min_motor_positions[i], self.max_motor_positions[i])

            # Debug
            # self.node.get_logger().info(f"final target pos = {self.target_position}")
            # self.node.get_logger().info("test")

            # Finally, set motor position
            # motors[i].setVelocity(target_position[i])
            self.motors[i].setPosition(self.target_position[i])
        
        msg = Float32MultiArray()
        msg.data = set(self.target_position.flatten())
        self.pub.publish(msg)
        #rclpy.spin_once(self.node)

'''
def main(args):
    rclpy.init()
    controller = SnakeRobotController(webots_node=args["robot_description"], properties=None)

    #while controller.robot.step(controller.timestep) != -1:
    while rclpy.ok():
        #rclpy.spin_once(controller)
        controller.step()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()
'''