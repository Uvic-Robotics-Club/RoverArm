RoverArm
This is the package that has everything with the RoverArm. 

The topics that this creates:
* /Arm/AngleVelocities
  * This is used for setting the manual speed for each joint
* /Arm/Diagnostics
  * This is used to see what is actually sent to the μC
* /Arm/Feedback
  * This is the angles from the Arm, in degrees
* /Arm/Goal
  * This is where the IK will calculate the angles for
* /Arm/Position
  * This is the angles that the arm will set to, defined by the IK

The topics that this package listens to is
* /arm_joy
