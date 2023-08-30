// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.XboxController;
import com.kauailabs.navx.frc.AHRS;





public class Robot extends TimedRobot {
  
  XboxController DriveController = new XboxController(0);
  DriveSystem Drive = new DriveSystem();
  AHRS GyroSensor = new AHRS();
  @Override
  public void teleopInit() {
    GyroSensor.reset();
  }
  @Override
  public void teleopPeriodic() {
    if(DriveController.getRightBumper()){
      Drive.Lock();
    }else{
    Drive.Execute(DriveController.getLeftX(), -DriveController.getLeftY(), DriveController.getRightX(), GyroSensor.getAngle());
    }
  }
}