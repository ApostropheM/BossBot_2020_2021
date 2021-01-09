These programs are made for use with FTC software and the BossBots team robot.

bossbot_top.png and bossbot_isometric.png are images of 3d models of the 2020 robot.
gamepad_config.png shows plans for controller workings.
Circuit_Diagram.png shows plans for hooking the robot up to the Expansion Hub.

Mecanum.java
This file contains basic driving code for the robot, designed around its mechanum wheels for use with xbox 360 or logitech controllers.
The while loop in runOpMode() repeatedly runs the drive function, so this program only allows driving. fd represents how much forward power is sent, subtracting the power sent to the left trigger (back) from the power sent to the front trigger (forwards).
Wheels 1 and 4 have their power inverted at the end of the function because the motor power determines clockwise vs counterclockwise. Depending on the setup, some of them need negative the normal power (on the left vs right sides, because the motors always are positioned facing outwards fom the inside of the robot).
The four wheels are diagonal, so moving forward requires positive motion on all motors. Turning right requires that wheels opposite each other turn in opposite directions. Drifting requires that diagonal wheels move together, so motors 1 and 3 move the opposite direction from 2 and 4 because of how the drift variable is incorporated. It only takes in the x value of the stick, so drifting is only left and right. Moving forward and backward still uses the triggers.
It uses caps on the power to keep it between -1 and 1.

Infra.java
This file contains java code used for the 2020-2021 robot's old design with infrared sensors. It uses a pair of sensor inputs to determine how many rings are in front of it for the autonomous portion of the FTC competition (0,1,4).

BossBot_20_21.java
This file is the current 2020-2021 file used for the teleop portion of FTC competitions.
runOpMode() creates variables for parts on the robot and prepares them, then sets its controls. The while loop checks repeatedly for inputs.
launcher() is a function to launch rings with less inputs, coordinating several parts to the robot. It's made to shoot from the line on the field. It checks where the ramp is, then moves it to the right position, moves the rings up, and starts up the launching motor.
The drive() function is slightly modified from mechanum.java.

Auton_20_21.java
The instance variables at the top declare all of the robot's parts, and then the motors and imu are set up.
In runOpMode(), the if(opModeIsActive) statement activates the auton sequence. (Because the infrared sensors are out, it currently doesn't check the number of rings.) It moves to the rings from a specific starting position on the firled, moves to the side and up to the line, fires its rings, puts down the wobble goal, and parks. The checks on numRings are to determine where to park.
drive() is the same as on Bossbot_20_21. fd, yaw, and drift are helper functions used to keep the robot on track without driver adjustment. fd() uses drive() and IMU sensors to make sure it goes forward. brake() brakes all the motors, and yaw() turns the robot until it reaches the angle argument.
fd() is a function similar to drive using imu to move around, unlike with teleop, which just uses drive. Yaw acts similarly. Brake and drift are helper functions that 

