package frc.robot.subsystems;


import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends SubsystemBase{

  public DriveSubsystem(){
    GyroSensor.calibrate();
  }



  public final SwerveModule BackRight = new SwerveModule(7, 8, Math.toRadians(270));
  public final SwerveModule FrontRight = new SwerveModule(3, 4, Math.toRadians(0));
  public final SwerveModule BackLeft = new SwerveModule(5, 6, Math.toRadians(180));
  public final SwerveModule FrontLeft = new SwerveModule(1, 2, Math.toRadians(90));
  
  AHRS GyroSensor = new AHRS();

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


  public double GetGyro(){
    return(GyroSensor.getAngle());
  }

  public void ResetGyro(){
    GyroSensor.reset();
  }

  public void Lock(){
    FrontRight.Run(0, Math.toRadians(45));
    BackRight.Run(0, Math.toRadians(-45));
    BackLeft.Run(0, Math.toRadians(45));
    FrontLeft.Run(0, Math.toRadians(-45));
  }

  public void Drive(double x1, double y1, double x2){
    StrafeDirection = Math.atan2(x1, y1) - Math.toRadians(GyroSensor.getAngle());
    StrafeMagnitude = Math.hypot(x1, y1);
    StrafeMagnitude = Math.min(StrafeMagnitude, 1) * DriveConstants.StrafePercent;
    //The line above is to make sure that the Magnitude isn't greater then 1
    StrafeX = Math.sin(StrafeDirection) * StrafeMagnitude;
    StrafeY = Math.cos(StrafeDirection) * StrafeMagnitude;
    
    RotateX = Math.cos(DriveConstants.TurnAngle) * Math.min(x2, 1) * DriveConstants.RotatePercent;
    RotateY = Math.sin(DriveConstants.TurnAngle) * Math.min(x2, 1) * DriveConstants.RotatePercent;
    PosFullX = StrafeX + RotateX;
    PosFullY = StrafeY + RotateY;
    NegFullX = StrafeX - RotateX;
    NegFullY = StrafeY - RotateY;


    FrontRight.Run(Math.hypot(PosFullX, NegFullY), Math.atan2(PosFullX, NegFullY));
    BackRight.Run(Math.hypot(NegFullX, NegFullY), Math.atan2(NegFullX, NegFullY));
    BackLeft.Run(Math.hypot(NegFullX, PosFullY), Math.atan2(NegFullX, PosFullY));
    FrontLeft.Run(Math.hypot(PosFullX, PosFullY), Math.atan2(PosFullX, PosFullY));
    
  }

  public double[] FullPosition = {0,0};
  public double RobotYPosition = 0;
  public double RobotXPosition = 0;

  @Override
  public void periodic(){

    //This gets the position of the robot and returns it as an array with the x and y value
    double[] FrontRightPosition = FrontRight.ModulePosition(GyroSensor.getAngle());
    double[] FrontLeftPosition = FrontLeft.ModulePosition(GyroSensor.getAngle());
    double[] BackRightPosition = BackRight.ModulePosition(GyroSensor.getAngle());
    double[] BackLeftPosition = BackLeft.ModulePosition(GyroSensor.getAngle());
    RobotXPosition = (FrontRightPosition[0] + 
    FrontLeftPosition[0] +
    BackLeftPosition[0] +
    BackRightPosition[0]) / 4;
    RobotYPosition = (FrontRightPosition[1] +
    FrontLeftPosition[1] +
    BackLeftPosition[1] +
    BackRightPosition[1]) / 4;
    FullPosition[0] = RobotXPosition;
    FullPosition[1] = RobotYPosition;
    
  }

  public double[] RobotPosition(){
    return(FullPosition);
  }

  public void ResetRobotPosition(){
    RobotYPosition = 0;
    RobotXPosition = 0;
    FrontRight.ResetModulePosition();
    BackRight.ResetModulePosition();
    BackLeft.ResetModulePosition();
    FrontLeft.ResetModulePosition();
  }
}
