Scripts named with "_test" are for modification testing or communications testing.
The launch file is currently configured for testing, to launch stocky in fully functioning mode:
  - Trade out "arm_control_test.ino" for "arm_control.ino"
  - Trade out "US_sensor_test.ino" for "tank_control.ino"
(Ensure these codes match with those currently loaded onto the Arduino)

Place all of these scripts into a catkin workspace and modify the .launch file to match what you are wanting to do (see above for reference).
once the workspace has been set up, follow the below steps:
  1. Ensure roscore is running then put it "roslaunch <your_catkin_workspace> capstone.launch
  2. Follow the prompts on the terminal for line thresholding (enter y to use existing range data, enter n to calibrate new data).
  3. Follow the prompts to threshold the light signal sensor (enter y to use existing data, n to calibrate new)
