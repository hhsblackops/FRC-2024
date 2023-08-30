// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {
  public static class DriveConstants {

    //This is how fast your robot will go forward and rotate.
    //THESE 2 VALUES COMBINED CAN NOT BE BIGGER THEN 1!!
    public static final double StrafePercent = 0.5;
    public static final double RotatePercent = 0.5;

    
    //From center of one wheel to the other, front to back
    public static final double RobotLength = 26.2; //inches

    //Also from center of wheel to the other, left to right
    public static final double RobotWidth = 26.2; //inches
    public static final double TurnAngle = Math.atan(RobotWidth/RobotLength); //Is in radians
  }
}
