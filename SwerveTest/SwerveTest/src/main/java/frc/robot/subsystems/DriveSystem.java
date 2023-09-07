package frc.robot.subsystems;


import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;


public class DriveSystem{

  public final SwerveModule BackRight = new SwerveModule(7, 8, Math.toRadians(270));
  public final SwerveModule FrontRight = new SwerveModule(3, 4, Math.toRadians(0));
  public final SwerveModule BackLeft = new SwerveModule(5, 6, Math.toRadians(180));
  public final SwerveModule FrontLeft = new SwerveModule(1, 2, Math.toRadians(90));
  


  public double StrafeDirection;
  public double StrafeMagnitude;

  public double StrafeX;
  public double StrafeY;
  public double RotateX;
  public double RotateY;
  public double PosFullX;
  public double PosFullY;
  public double NegFullX;
  public double NegFullY;

  public void Lock(){
    FrontRight.Run(0, Math.toRadians(45));
    BackRight.Run(0, Math.toRadians(-45));
    BackLeft.Run(0, Math.toRadians(45));
    FrontLeft.Run(0, Math.toRadians(-45));
  }

  public void Drive(double x1, double y1, double x2, double Gyro){
    StrafeDirection = Math.atan2(x1, y1) - Math.toRadians(Gyro);
    StrafeMagnitude = Math.hypot(x1, y1) * DriveConstants.StrafePercent;
    StrafeX = Math.sin(StrafeDirection) * StrafeMagnitude;
    StrafeY = Math.cos(StrafeDirection) * StrafeMagnitude;
    RotateX = Math.cos(DriveConstants.TurnAngle) * x2 * DriveConstants.StrafePercent;
    RotateY = Math.sin(DriveConstants.TurnAngle) * x2 * DriveConstants.StrafePercent;
    PosFullX = StrafeX + RotateX;
    PosFullY = StrafeY + RotateY;
    NegFullX = StrafeX - RotateX;
    NegFullY = StrafeY - RotateY;


    FrontRight.Run(Math.hypot(PosFullX, NegFullY), Math.atan2(PosFullX, NegFullY));
    BackRight.Run(Math.hypot(NegFullX, NegFullY), Math.atan2(NegFullX, NegFullY));
    BackLeft.Run(Math.hypot(NegFullX, PosFullY), Math.atan2(NegFullX, PosFullY));
    FrontLeft.Run(Math.hypot(PosFullX, PosFullY), Math.atan2(PosFullX, PosFullY));
    
  }

  public double[] RobotPosition(double Gyro){
    //This gets the position of the robot and returns it as an array with the x and y value
    double[] FrontRightPosition = FrontRight.ModulePosition(Gyro);
    double[] FrontLeftPosition = FrontLeft.ModulePosition(Gyro);
    double[] BackRightPosition = BackRight.ModulePosition(Gyro);
    double[] BackLeftPosition = BackLeft.ModulePosition(Gyro);
    double RobotXPosition = (FrontRightPosition[0] + 
    FrontLeftPosition[0] +
    BackLeftPosition[0] +
    BackRightPosition[0]) / 4;
    double RobotYPosition = (FrontRightPosition[1] +
    FrontLeftPosition[1] +
    BackLeftPosition[1] +
    BackRightPosition[1]) / 4;
    double[] FullPosition = {RobotXPosition, RobotYPosition};
    return(FullPosition);
  }


}