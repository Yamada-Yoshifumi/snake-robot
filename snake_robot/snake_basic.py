"""
snake_basic controller with ROS2 

Code written by ourselves unless otherwise credited.

"""

#********* IMPORTS *********#
# Import basic py libs
import cv2 as cv2   # opencv
import os, datetime
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

# IMAGE CAPTURE RATE FOR CAMERA
# Write "default" to use default rate of 32 ms, but here we use slower rates as per paper
CAMERA_RATE = 32  # paper used 2500ms
# CAMERA_RATE = 'default'    

# FOR DEBLURRING
# Define Point Spread Fn hyperparameters
SIZE = 1
SNR = 10
ANGLE = 90  # This angle will be adjusted according to horizon detected


# This class includes all key functions

class SnakeRobotController:

    
    def init(self, webots_node, properties):

        #********* INITIALISATION *********#
        """
        Initialise (mostly) tuning and global variables for the class
        """

        # Turn snake leg mode on or off
        # Note: salamander doesn't work without legs due to lack of friction
        # True to keep legs, False to extend legs
        self.snake_mode = False   

        # Deblur filter on/off
        self.deblur_filter_on = True

        self.robot = webots_node.robot

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())       # in milliseconds
        # Record when the previous image was taken
        self.previous_sample_time = int(self.robot.getTime())    # in seconds

        rclpy.init(args=None)
        self.node = rclpy.create_node('snake_basic')
        self.pub = self.node.create_publisher( Float32MultiArray, 'motor', 10)
        self.node.get_logger().info(f"simulation timestep = {self.timestep} ms")    # DEBUG

        # Track elapsed time
        self.time_elapsed = 0

        # Create list to store blurriness value of images
        self.blur_list_org = []
        if self.deblur_filter_on:
            self.blur_list_deblur = []

        # Initial locomotion values
        self.spine_offset = 0.0  # this controls turning. negative vals to turn left, positive turns right
        self.amplitude = 1.0     # Gait eqn amplitude
        self.phase = 0.0         # current locomotion phase
        self.walk_ampl = 0.6     # Walk amplitude in rads (for body swaying)
        self.freq = 1/1        # Gait freq in Hz. Note: freq = 1/T. Originally =1
        self.node.get_logger().info(f"Gait period = {1/self.freq} s and freq = {self.freq} Hz")

        # Initialise list to store target motor positions
        self.target_position = np.zeros(NUM_MOTORS)

        
        #********* START CAMERA *********#
        """
        Command to start camera and get image data from it
        """
        ## NEW camera refresh rate code

        # Start camera
        self.camera = self.robot.getDevice('camera')

        ## NEWER
        # Select camera refresh rate
        cam_rate = self.timestep # just keep it at default 32ms

        # Compute number of cycles before camera refreshes
        if isinstance(CAMERA_RATE, int):
            # Camera will refresh at number of cycles closest to requested rate
            closest_rate = np.floor(cam_rate * np.floor( CAMERA_RATE / cam_rate ))
            self.cam_cycle = int(closest_rate / cam_rate)
        else:
            self.cam_cycle = 1 # camera refreshes at every simulation step, the fastest possible

        # image capture rate of camera
        self.node.get_logger().info(f"camera takes one pic every {self.cam_cycle * cam_rate} ms")

        # Dimensions
        self.camera.enable(cam_rate)
        img_width = self.camera.getWidth()
        img_height = self.camera.getHeight()
        self.node.get_logger().info(f"dims of img: w={img_width}, h={img_height}")


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


    #********* DE-BLURRING FUNCTIONS *********#

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
        img = np.asarray(cameraData, dtype=np.uint8).reshape((80, 120, 3))
        #img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)

        
        # print(f"cv ver = {cv2.__version__ }")     # Debug
        #img_cv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        img_cv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #img_cv = cv2.rotate(img_cv, cv2.ROTATE_90_CLOCKWISE)
        #img_cv = cv2.flip(img_cv, 1)

        return img_cv
    

    def calc_blur(self, img_cv):
        """
        Calculates its blurriness using variance of Laplacian
            img_cv: OpenCV object in HSV format
            orginal_img: boolean for file naming purposes
        """
        # Calculates blurriness
        # First computes the Laplacian (2nd derivative), then return variance
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_HSV2BGR)
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        var_Laplc = cv2.Laplacian(img_cv, cv2.CV_64F).var()

        return var_Laplc

    # For image deblurring
    def blur_edge(self, img, d=31):
        """
        Takes in an image and blurs the edges. An optional step in deblur pipeline
        Code adapted from: https://github.com/npinto/opencv/blob/master/samples/python2/deconvolution.py
        """
        h, w  = img.shape[:2]
        img_pad = cv2.copyMakeBorder(img, d, d, d, d, cv2.BORDER_WRAP)
        img_blur = cv2.GaussianBlur(img_pad, (2*d+1, 2*d+1), -1)[d:-d,d:-d]
        y, x = np.indices((h, w))
        dist = np.dstack([x, w-x-1, y, h-y-1]).min(-1)
        w = np.minimum(np.float32(dist)/d, 1.0)
        return img*w + img_blur*(1-w)


    # For image deblurring
    def motion_kernel(self, angle, d, sz=65):
        """
        Creates a motion kernel based on Point Spread Fn.
        Code adapted from: https://github.com/npinto/opencv/blob/master/samples/python2/deconvolution.py
        """
        kern = np.ones((1, d), np.float32)
        c, s = np.cos(angle), np.sin(angle)
        A = np.float32([[c, -s, 0], [s, c, 0]])
        sz2 = sz // 2
        A[:,2] = (sz2, sz2) - np.dot(A[:,:2], ((d-1)*0.5, 0))
        kern = cv2.warpAffine(kern, A, (sz, sz), flags=cv2.INTER_CUBIC)
        return kern

    # For image deblurring
    def deblur(self, img, angle, size, snr):
        """
        Img deblurring pipeline. Returns deblurred img
        Code adapted from: https://github.com/npinto/opencv/blob/master/samples/python2/deconvolution.py
        """
        # Blur edge and DFT
        img = np.float32(img)/255.0
        # img = blur_edge(img)
        IMG = cv2.dft(img, flags=cv2.DFT_COMPLEX_OUTPUT)

        # Define PSF
        ang = np.deg2rad(angle)  # Angle of PSF, range 0-180, def 135
        d = size                 # size, range 0-50, def 22
        noise = 10**(-0.1* snr )  # SNR, range 0-50, def 25

        # Define Motion kernel
        psf = self.motion_kernel(ang, d)
        # plt.imshow(psf)

        psf /= psf.sum()
        psf_pad = np.zeros_like(img)
        kh, kw = psf.shape
        psf_pad[:kh, :kw] = psf
        PSF = cv2.dft(psf_pad, flags=cv2.DFT_COMPLEX_OUTPUT, nonzeroRows = kh)
        PSF2 = (PSF**2).sum(-1)
        iPSF = PSF / (PSF2 + noise)[...,np.newaxis]
        RES = cv2.mulSpectrums(IMG, iPSF, 0)
        res = cv2.idft(RES, flags=cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT )
        res = np.roll(res, -kh//2, 0)
        res = np.roll(res, -kw//2, 1)

        return (res*255).astype(np.uint8)
    

    def split_deblur_merge(self, img):
        """
        Splits image up into BGR, runs deblur on each channel, 
        then merges it back as deblurred 3 channel img. 
        
        It is necessary to split images as this filter implementation
        only works on a single channel at once. Experimentally,
        splitting images up and merging yields similar performance
        to that of a single-channel grayscale image.
        """

        # convert HSV to BGR
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

        # Split img into channels
        img_b,img_g,img_r = cv2.split(img)

        # Define Point Spread Fn parameters
        size = SIZE
        snr = SNR
        angle = ANGLE

        # Angle determind using horizon tilt
        try:
            angle = self.calc_horizon_angle(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            print(f"Horizon angle = {angle}")   # debug
        except: 
            angle = 0
        angle += 90

        # Run deblur on each channel
        img_out = []
        for channel in [img_b,img_g,img_r]:
            res = self.deblur(channel, angle, size, snr)
            img_out.append(res)

        # Merge channels back
        img_merged = cv2.merge(img_out)

        # Run a median filter to remove weird coloured pixels
        # img_merged = cv2.medianBlur(img_merged, 1)
        # img_merged = cv2.fastNlMeansDenoisingColored(img_merged)

        # Increase brightness
        delta_contrast=1.0
        delta_brightness=8
        img_merged = cv2.convertScaleAbs(img_merged, alpha=delta_contrast, beta=delta_brightness)
        
        # Convert back to HSV
        img_merged = cv2.cvtColor(img_merged, cv2.COLOR_BGR2HSV)

        return img_merged
    

    # Horizon detection function
    def calc_horizon_angle(image_grayscale):
        """
        Takes in grayscale image and gives horizon tilt in degrees
        """
        edges = cv2.Canny(image_grayscale,100,300)  # 100, 300 are arbitarily determined

        # Find max value in each column
        horz_xcoords = np.amax(edges, 0)

        # Find max and min x-coordinates
        horz_xcoords_pos = np.where(horz_xcoords==255)[0]
        min_x = np.amin(horz_xcoords_pos)
        max_x = np.amax(horz_xcoords_pos)

        # Find y_coords of horiz
        min_y = np.where(edges[:,min_x]==255)[0][0]
        max_y = np.where(edges[:,max_x]==255)[0][0]

        # Calc angle in rads
        horz_angle = np.arctan((max_y-min_y)/(max_x-min_x))

        return np.rad2deg(horz_angle)   # convert to deg



    #********* NAVIGATION FUNCTIONS  *********#  

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
        mid_h = int(np.floor(h / 2)) + int(np.floor(h / 20))  # Add a tolerance because of head's vertical tilting

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


    def collision_check(self, mask, points, h, w):
        """
        Calculate how long can each trajectory (represented by the end points) can travel
        in vertical direction in image space without collision.
        Return the number of the trajectory that can travel the longest vertical length.
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
        length = np.array([0, 0, 0])        # Record (vertical) length of trajectories
        end_flag = [False, False, False]    # Record whether a trajecory has encountered an obstacle
        step1 = (points[1][1] - mid_w) / (h - mid_h)
        step2 = (points[2][1] - mid_w) / (h - mid_h)

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
                    length[1] += 1
                else:
                    end_flag[1] = True
            # Check third trajectory (turning right)
            if not end_flag[2]:
                column = int(np.floor(column_start + step2*i))
                if mask[row, column]:
                    length[2] += 1
                else:
                    end_flag[2] = True

        # Return the number of the trajectory with the longest vertical length
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
        '''
        # Debugging!
        img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        cv2.namedWindow("Input image")
        cv2.imshow("Input image", img_RGB)
        cv2.waitKey(0)
        cv2.namedWindow("Mask")
        cv2.imshow("Mask", mask)
        cv2.waitKey(0)

        # Save some images for illustration
        if choice == 2:
            img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
            cv2.imwrite("right_turn_image.png", img_RGB)
            self.node.get_logger().info(f"Image saved!")
            cv2.imwrite("right_turn_mask.png", mask)
            self.node.get_logger().info(f"Mask saved!")
        if choice == 1:
            img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
            cv2.imwrite("left_turn_image.png", img_RGB)
            self.node.get_logger().info(f"Image saved!")
            cv2.imwrite("left_turn_mask.png", mask)
            self.node.get_logger().info(f"Mask saved!")
        if choice == 0:
            img_RGB = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
            cv2.imwrite("forward_image.png", img_RGB)
            self.node.get_logger().info(f"Image saved!")
            cv2.imwrite("forward_mask.png", mask)
            self.node.get_logger().info(f"Mask saved!")
        '''
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


    #********* HELPER FUNCTIONS *********#
    """
    Put helper functions here. E.g., Gait equation
    """
    def restrict(self, target_pos, min_pos, max_pos):
        """
        Clamps motor position to within min/max values 
        """
        if (min_pos == 0 and max_pos == 0):     # weird case, just remain unchanged
            return target_pos
        else:   # clamps value to limit
            return max(min_pos, min(max_pos, target_pos))
    
    #********* MAIN CONTROL FUNCTION *********#
    """
    Defines actions for each control timestep
    """
    def step(self):

        ## ***** 1. Read the camera ***** ##

        # Initially no image
        img_cv = None

        # Get an image from the camera and convert to OpenCV format
        # This only takes place every specific number of cycles depending on requested sampling rate
        img_capture_flag = self.time_elapsed % (self.cam_cycle * self.timestep) == 0
        
        if img_capture_flag:
            # calc blur score
            img_cv = self.get_image()
            blur_score_org = self.calc_blur(img_cv)
            self.blur_list_org.append(blur_score_org)

            # ## Deblurring step
            if self.deblur_filter_on:
                img_cv = self.split_deblur_merge(img_cv)

                # calc blur score
                blur_score_deblur = self.calc_blur(img_cv)
                self.blur_list_deblur.append(blur_score_deblur)

        # Calculate mean blurriness scores and write to file
        if img_capture_flag:
            mean_blur_org = np.mean(np.array(self.blur_list_org))

            if self.deblur_filter_on:
                mean_blur_deblur = np.mean(np.array(self.blur_list_deblur))
                blur_msg = f"Mean blur, before: {mean_blur_org}, after deblur: {mean_blur_deblur}"
            #else:
                #blur_msg = f"Mean blur: {mean_blur_org}. Deblur filter is off."
            self.node.get_logger().info(blur_msg)


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

        # NEW: update time elapsed
        self.time_elapsed += self.timestep

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