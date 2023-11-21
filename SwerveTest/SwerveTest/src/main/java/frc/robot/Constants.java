// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {
  public static class DriveConstants {
    //This is how fast your robot will go forward and rotate.
    //THESE 2 VALUES COMBINED CAN NOT BE BIGGER THEN 1!!
    public static final double MaxStrafeSpeed = 7; //feet per second
    public static final double MaxRotateSpeed = Math.PI / 10; //radians per second

    //The unit of measurement for the next 2 lines doesn't matter, as long as they're both the same.
    public static final double RobotLength = 20; //inches
    public static final double RobotWidth = 20; //inches

    /*if your robot isn't a perfect square, the best angle to turn the robot is not exactly
    every wheel at 45 degrees. This next line calculates it for you.*/
    public static final double TurnAngle = Math.atan(RobotWidth/RobotLength);

    /* This is the number that will be counted as "rotations" in the driving wheel.
    We chose this number because it is how many motor rotations will make the robot go foward 
    approximately 1 foot.*/
    public static final double PinionTeethNumber = 14;
    public static final double MotorToWheelGearRatio = (45.0 * 22) / (PinionTeethNumber * 15);
    public static final double WheelDiameter = 0.25; //feet
    public static final double DrivingPositionFactor = ((WheelDiameter * Math.PI) / MotorToWheelGearRatio);

    public static final double DrivingVelocityFactor = ((WheelDiameter * Math.PI) / MotorToWheelGearRatio) / 60;
  }


  public static class AutoConstants {

  }
}
