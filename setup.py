from setuptools import setup
package_name = 'snake_robot'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/snake_robot.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/snake_basic.wbt']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/controllers/snake_basic', ['controllers/snake_basic/controller.py', 'controllers/snake_basic/snake_basic.py']))
data_files.append(('share/' + package_name + '/libraries', ['libraries/controller.py', 'libraries/vehicle.py']))
data_files.append(('share/' + package_name + '/resource', ['resource/salamander.urdf']))

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=data_files,
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'trajectory_recording = snake_robot.trajectory_recording:main',
             'my_robot_driver = snake_robot.my_robot_driver:main',
     ],
   },
)