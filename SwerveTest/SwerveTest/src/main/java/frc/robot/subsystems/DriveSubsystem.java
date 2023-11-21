package frc.robot.subsystems;


import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;



public class DriveSubsystem extends SubsystemBase{

  public DriveSubsystem(){
    GyroSensor.calibrate();
  }



  public final SwerveModule BackRight = new SwerveModule(19, 18, Math.toRadians(270));
  public final SwerveModule FrontRight = new SwerveModule(2, 3, Math.toRadians(0));
  public final SwerveModule BackLeft = new SwerveModule(17, 16, Math.toRadians(180));
  public final SwerveModule FrontLeft = new SwerveModule(4, 1, Math.toRadians(90));
  
  AHRS GyroSensor = new AHRS();



  public double GetGyroDegrees(){
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
  double kP = 0.1;
  double kF = 0;
  public void Drive(double x1, double y1, double x2){
    /*SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kF", kF);
    kP = SmartDashboard.getNumber("kP", kP);
    kF = SmartDashboard.getNumber("kF", kP);*/

    FrontRight.PIDTuning(kP, kF);
    FrontLeft.PIDTuning(kP, kF);
    BackRight.PIDTuning(kP, kF);
    BackLeft.PIDTuning(kP, kF);


    double ControllerStarfeX = Math.min(x1, 1);
    double ControllerStrafeY = Math.min(y1, 1);
    double ControllerRotate = Math.min(x2, 1);
    double StrafeDirection = Math.atan2(ControllerStarfeX, ControllerStrafeY) - Math.toRadians(GyroSensor.getAngle());
    double StrafeMagnitude = Math.hypot(ControllerStarfeX, ControllerStrafeY) * DriveConstants.MaxStrafeSpeed;
    double StrafeX = Math.sin(StrafeDirection) * StrafeMagnitude;
    double StrafeY = Math.cos(StrafeDirection) * StrafeMagnitude;
    
    double RotateX = Math.cos(DriveConstants.TurnAngle) * ControllerRotate * ((Math.hypot(DriveConstants.RobotLength, DriveConstants.RobotWidth) / 2 ) * DriveConstants.MaxRotateSpeed);
    double RotateY = Math.sin(DriveConstants.TurnAngle) * ControllerRotate * ((Math.hypot(DriveConstants.RobotLength, DriveConstants.RobotWidth) / 2 ) * DriveConstants.MaxRotateSpeed);
    double PosFullX = StrafeX + RotateX;
    double PosFullY = StrafeY + RotateY;
    double NegFullX = StrafeX - RotateX;
    double NegFullY = StrafeY - RotateY;



    FrontRight.Run(Math.hypot(PosFullX, NegFullY), Math.atan2(PosFullX, NegFullY));
    BackRight.Run(Math.hypot(NegFullX, NegFullY), Math.atan2(NegFullX, NegFullY));
    BackLeft.Run(Math.hypot(NegFullX, PosFullY), Math.atan2(NegFullX, PosFullY));
    FrontLeft.Run(Math.hypot(PosFullX, PosFullY), Math.atan2(PosFullX, PosFullY));
    
    /*double Direction = Math.toRadians(0);
    double Speed = 1;
    FrontRight.Run(Speed, Direction);
    BackRight.Run(Speed, Direction);
    BackLeft.Run(Speed, Direction);
    FrontLeft.Run(Speed, Direction);*/


    
  }

  public double[] FullPosition = {0,0,0};
  public double RobotYPosition = 0;
  public double RobotXPosition = 0;
  public double RobotVelocity = 0;

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

    RobotVelocity = (FrontRightPosition[2] +
    FrontLeftPosition[2] +
    BackLeftPosition[2] +
    BackRightPosition[2]) / 4;

    FullPosition[0] = RobotXPosition;
    FullPosition[1] = RobotYPosition;
    FullPosition[2] = RobotVelocity;
    
    
  }



  public double[] GetRobotPosition(){
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
