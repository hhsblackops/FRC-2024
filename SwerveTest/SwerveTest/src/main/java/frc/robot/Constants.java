// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {
  public static class DriveConstants{
    //This is how fast your robot will go forward and rotate.
    //THESE 2 VALUES COMBINED CAN NOT BE BIGGER THEN 1!!
    public static final double MaxStrafeSpeed = 5; //feet per second
    public static final double MaxRotateSpeed = 1; //radians per second

    /*These are the values for strafing and rotating to positions when the robot is 
    driving without the controller.*/
    public static final double StrafeP = 3;
    public static final double StrafeI = 0;
    public static final double StrafeD = 0;

    public static final double RotateP = 4.5;
    public static final double RotateI = 0;
    public static final double RotateD = 0;

    //These are all the CAN IDs for the Sparks of the modules.
    public static final int FrontRightDrivingID = 18;
    public static final int FrontRightTurningID = 19;
    public static final int FrontLeftDrivingID = 7;
    public static final int FrontLeftTurningID = 6;
    public static final int BackRightDrivingID = 16;
    public static final int BackRightTurningID = 17;
    public static final int BackLeftDrivingID = 5;
    public static final int BackLeftTurningID = 20;

    //next 2 must be measured from the center of the wheels.
    public static final double RobotLength = 17; //inches front to back
    public static final double RobotWidth = 17; //inches left to right

    //These next lines are for the PID values for the wheels.
    public static final double MovingP = 0.1;
    public static final double MovingI = 0;
    public static final double MovingD = 0;
    public static final double MovingFF = 0.065;

    public static final double TurningP = 1;
    public static final double TurningI = 0;
    public static final double TurningD = 0;


    /*if your robot isn't a perfect square, the best angle to turn the robot is not exactly
    every wheel at 45 degrees. This next line calculates it for you.*/
    public static final double TurnAngle = Math.atan(RobotWidth/RobotLength);

    /* This is the number that will be counted as "rotations" in the driving wheel.
    We chose this number because it is how many motor rotations will make the robot go foward 1 foot.*/
    public static final double PinionTeethNumber = 14;
    public static final double MotorToWheelGearRatio = (45.0 * 22) / (PinionTeethNumber * 15);
    public static final double WheelDiameter = 0.25; //feet
    public static final double DrivingPositionFactor = ((WheelDiameter * Math.PI) / MotorToWheelGearRatio);

    public static final double DrivingVelocityFactor = ((WheelDiameter * Math.PI) / MotorToWheelGearRatio) / 60;

    public static final int HorizontalChannelA = 0;
    public static final int HorizontalChannelB = 1;

    public static final int VerticalChannelA = 3;
    public static final int VerticalChannelB = 4;

  }

  public static class LEDConstants{
    public static final int LEDPort = 8;
  }
  public static class AutoConstants{
    public static final double[] TestPath = {0, 0,
    1, 1,
    2, 2
    };
  }
}
