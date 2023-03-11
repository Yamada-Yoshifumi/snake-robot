# Documentation:

To start webots simulation with ROS, build the package, and do:

<code>ros2 launch snake_robot snake_robot.launch.py</code>

Code to source and build
<code>
colcon build --symlink-install
source install/setup.bash
</code>

## Salamnder robot
https://cyberbotics.com/doc/guide/salamander

This can get tricky later because we need to keep controllers/snake_basic/snake_basic.py up to date with snake_robot/snake_basic.py, and vice versa. There is currently no way to use the same piece of .py script for both pure webots and webots with ROS.

To start webots simulation with ROS, build the package, and do:

<code>ros2 launch snake_robot snake_robot.launch.py</code>

Code to source and build
<code>
colcon build --symlink-install
source install/setup.bash
</code>

## Trajectory sampling

When a new image is captured (after `camera_rate` amount of time elapses), the robot chooses a (local) trajectory based on the image.  
Each trajectory is controlled by the setting of `spine_offset`.  
The trajectories are checked in sequence, in the following order:
1. Go straight forward (`spine_offset = 0`)
2. Turn left (`spine_offset = -0.3`)
3. Turn left (`spine_offset = 0.3`)
4. Sharp turn left (`spine_offset = -0.6`)
, and the trajectory which has the **longest collision-free** length will be chosen.

**Note:** Previously, I implemented this by only counting the **vertical** length in the image (i.e., along y-axis in image frame). This has been corrected to use the **true** length (still approximated by straight lines, though). This matches with the paper and avoids some collisions, but the downside is that salamander tends to do turnings all the time.  
If you want the previous behaviour, you can pass `vertical_length=True` to `collision_check()` function. 

## Collision checking

For each trajectory, I logged the initial location of the robot and its location after `camera_rate` (1s in my setting). (*My approach was kind of cheating here because I set the salamander as `Supervisor` which has full access to ground-truth locations; but this was merely for recording the trajectory and the robot does not need this information in deployment.*)  
The second location (end-point of the trajectory) is projected onto to the image plane of the intial location (if the robot were still there). Webots cameras seem to be regular OpenGL ones, and I followed the instructions here:
* https://stackoverflow.com/questions/61555182/webot-camera-default-parameters-like-pixel-size-and-focus
* http://www.songho.ca/opengl/gl_projectionmatrix.html

I calculated the projection manually (I could have written Python code...) in the `data.xlsx` Excel sheet. (*It is NOT arranged in a clean and tidy way, so let me know if you have questions...*)

The collision checking then is basically checking whether the projected pixel is 1 or 0 in the ground-segmented image. Some special cases:
* 1st trajectory: Assume the head is at the same pose/height in every frame, its projection should always be on the middle horizontal line (i.e., horizon). If moving forward, it should also have no horizontal shift, so it must be the central pixel. No need to record trajecory for this case.
* 4th trajectory: This causes the salamander to take a sharp turn and its end point is out of the viewing frustrum at the old location, so no need to (or actually, we cannot) test for this case. This is used for emergency obstacle avoidance (when the first three options all fail).

### Extension?

The authors who built the Salamandra were trying to mimic real salamanders for biology research. As one experiment, they studied how body wave amplitude and offset affects the trajectory curvature during turnning on ground. It seems like they can fit a circle with the trajetory (formed by the center of mass). We might be able to fit a mathematicall model, so that we can predict the end location for any offset without manually testing each one.  
Paper: https://ieeexplore.ieee.org/document/6416074 (You would probably be interested in "VII. CHARACTERIZATION OF THE BASIC GAITS" - "B. Walking" - "3) Experiment C, Turning on Ground:")

## Ground segmentation


Currently, this is simply done by taking a small region at the bottom middle of the image as a reference, and using its average colour as the threshold. The thresholding is done in HSV space.

## Helper functions

There is a helper script `overlay.py` to overlay the trajectories onto the captured images/masks, which can be used for illustrations (as in our presentation). The call signature is:
```
python3 <img_path> <num_of_selected_trajectory>
```
, where `<img_path>` is where you store the image/mask, and `<num_of_selected_trajectory>` is the index of the chosen trajectory (you can get this from the log, and the chosen one will be coloured green; if you don't want this in the illustration, just pass a number other than $\{0,1,2\}$).
