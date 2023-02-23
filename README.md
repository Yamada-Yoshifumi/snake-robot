# Documentation:

## Salamnder robot
https://cyberbotics.com/doc/guide/salamander

## General webots robot:
https://cyberbotics.com/doc/reference/robot?tab-language=python

## Webots camera module:
https://cyberbotics.com/doc/reference/camera?tab-language=python

## Webots motor module:
https://cyberbotics.com/doc/reference/camera?tab-language=python

## Use:

To start webots simulation without ROS, simply open the worlds/snake_basic.wbt file using Webots. You will need to change the code block in .wbt files:

<code>
  Salamander {
    translation 0 0 0.04
    rotation 0 0 1 2.35619
    controller "&lt;extern&gt;"
    name "salamander"
    extensionSlot [
      Camera {
        fieldOfView 1.22
        width 120
        height 80
      }
    ]
  }
</code>
  
To:

<code>
  Salamander {
    translation 0 0 0.04
    rotation 0 0 1 2.35619
    controller "snake_basic"
    name "salamander"
    extensionSlot [
      Camera {
        fieldOfView 1.22
        width 120
        height 80
      }
    ]
  }
</code>

This can get tricky later because we need to keep controllers/snake_basic/snake_basic.py up to date with snake_robot/snake_basic.py, and vice versa. There is currently no way to use the same piece of .py script for both pure webots and webots with ROS.

To start webots simulation with ROS, build the package, and do:

<code>ros2 launch snake_robot snake_robot.launch.py</code>

Code to source and build
<code>
colcon build --symlink-install
source install/setup.bash
</code>
