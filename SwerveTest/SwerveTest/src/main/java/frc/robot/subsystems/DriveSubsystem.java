package frc.robot.Subsystems;


import frc.robot.Constants.DriveConstants;
import frc.robot.Modules.SwerveModule;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;


public class DriveSubsystem extends SubsystemBase{

  //initialize each of the swervemodules with their offsets and ID numbers.
  public final SwerveModule BackRight = new SwerveModule(DriveConstants.BackRightTurningID, DriveConstants.BackRightDrivingID, Math.toRadians(270));
  public final SwerveModule FrontRight = new SwerveModule(DriveConstants.FrontRightTurningID, DriveConstants.FrontRightDrivingID, Math.toRadians(0));
  public final SwerveModule BackLeft = new SwerveModule(DriveConstants.BackLeftTurningID, DriveConstants.BackLeftDrivingID, Math.toRadians(180));
  public final SwerveModule FrontLeft = new SwerveModule(DriveConstants.FrontLeftTurningID, DriveConstants.FrontLeftDrivingID, Math.toRadians(90));
  
  AHRS GyroSensor = new AHRS();

  Encoder HorizontalOdometryEncoder = new Encoder(DriveConstants.HorizontalChannelA, DriveConstants.HorizontalChannelB, false, EncodingType.k4X);
  Encoder VerticalOdometryEncoder = new Encoder(DriveConstants.VerticalChannelA, DriveConstants.VerticalChannelB, false, EncodingType.k4X);

  
  
  public DriveSubsystem(){
    GyroSensor.calibrate();

    HorizontalOdometryEncoder.reset();
    HorizontalOdometryEncoder.setDistancePerPulse(2 * Math.PI);

    VerticalOdometryEncoder.reset();
    VerticalOdometryEncoder.setDistancePerPulse(2 * Math.PI);
    
  }
  
  public double GetGyroDegrees(){
    //This function just reuturns the heading of the robot in degrees.
    return(GyroSensor.getAngle());
  }

  public void ResetGyro(){
    //This one resets the gyro sensor to 0
    GyroSensor.reset();
  }

  public void Lock(){
    /*This function sets all of the wheels to point toward the center of the robot,
    which makes it really hard for our robot to be moved or pushed.*/
    FrontRight.Run(0, Math.toRadians(45));
    BackRight.Run(0, Math.toRadians(-45));
    BackLeft.Run(0, Math.toRadians(45));
    FrontLeft.Run(0, Math.toRadians(-45));
  }


  
  public void Drive(double ControllerStarfeX, double ControllerStrafeY, double ControllerRotate){
    /*This is the function that takes the values of what a controller would put in. The x1 variable 
    represents the movement you want the robot to make left (-) to right (+) on a scale of -1 to 1. y1 is 
    the same but foward (+) to backward (-). x2 is the rotate you want also on a scale from -1 to 1 where
    - is rotate left and + is rotate right.*/

    
    /*The way that the serve drive math works is we take the vector of our strafe movement (left,
    right, forward, backward) and the vector of our rotate (rotate right/left) and we add them together.
    When we set it to the added velocity then it will give us full control of the robot,
    where we can strafe and rotate at the same time.*/

    /*The following if statement just makes sure that we aren't getting a value outside of the
    domain we want, because then it would spin faster then we have set the max for it to.*/
    if(ControllerRotate < 0){
      ControllerRotate = Math.max(ControllerRotate, -1);
    }else{
      ControllerRotate = Math.min(ControllerRotate, 1);
    }
    //Now we put the x and y components of the strafe vector to a direction and magnitude.
    //First we get the direction subtracting our heading from the gyro sensor to make the drive headless
    double StrafeDirection = Math.atan2(ControllerStarfeX, ControllerStrafeY) - Math.toRadians(GyroSensor.getAngle());
    /*Then we get the magnitude and multiply if by the maximum speed we want. We use the min function becase 
    the biggest we want the magnitude to be before we multiply it by the max speed is 1.*/
    double StrafeMagnitude = Math.min(1, Math.hypot(ControllerStarfeX, ControllerStrafeY)) * DriveConstants.MaxStrafeSpeed;
    /*Now we take this perfected direction and magnitude and change them back into X and Y components
    so we can add them to the rotating.*/ 
    double StrafeX = Math.sin(StrafeDirection) * StrafeMagnitude;
    double StrafeY = Math.cos(StrafeDirection) * StrafeMagnitude;
    
    /*We then get the X and Y components. The ideal angle to turn the wheel when rotating is perpendicular
    to the center of the robot. The next 2 lines get us these ideals angles.*/
    double RotateX = Math.cos(DriveConstants.TurnAngle) * ControllerRotate * ((Math.hypot(DriveConstants.RobotLength, DriveConstants.RobotWidth) / 2 ) * DriveConstants.MaxRotateSpeed);
    double RotateY = Math.sin(DriveConstants.TurnAngle) * ControllerRotate * ((Math.hypot(DriveConstants.RobotLength, DriveConstants.RobotWidth) / 2 ) * DriveConstants.MaxRotateSpeed);

    /*This next part adds the strafe and rotating vectors together. We have to add and subtract
    so many of them together because the wheel's direction are all different when we rotate the robot.
    For example, if the front right wheel were set to spin the robot right, then the vector of that wheel
    would have a positive X component, and a negative Y component. However, we would want the back right
    wheel to have a both compontents negative. As you can see in the next set of lines after this one,
    we set these +/- values to the wheels*/
    double PosFullX = StrafeX + RotateX;
    double PosFullY = StrafeY + RotateY;
    double NegFullX = StrafeX - RotateX;
    double NegFullY = StrafeY - RotateY;

    /*We then turn these components back into direction and magnitude, telling the wheels specificically
    which way to rotate (explained in the last comment), and set each module to that vector.*/
    FrontRight.Run(Math.hypot(PosFullX, NegFullY), Math.atan2(PosFullX, NegFullY));
    BackRight.Run(Math.hypot(NegFullX, NegFullY), Math.atan2(NegFullX, NegFullY));
    BackLeft.Run(Math.hypot(NegFullX, PosFullY), Math.atan2(NegFullX, PosFullY));
    FrontLeft.Run(Math.hypot(PosFullX, PosFullY), Math.atan2(PosFullX, PosFullY));
  }

  

  //Set each part of our position to initially start at 0
  double[] ModulePosition = {0, 0, 0};

  double[] OdometryPodPosition = {0, 0, 0};

  double CurrentOdometryPodX;
  double CurrentOdometryPodY;

  double PastOdometryPodX = 0;
  double PastOdometryPodY = 0;

  @Override
  public void periodic(){
    /*This next little chunk gives us the position of the robot based on the relative encoders in the motors.
    It takes all of the values for x position, y position, and velocity of each module and just takes the
    average of all of them. The velocity for this is only accurate when the robot is only strafing though.*/
    double[] FrontRightPosition = FrontRight.ModulePosition(GyroSensor.getAngle());
    double[] FrontLeftPosition = FrontLeft.ModulePosition(GyroSensor.getAngle());
    double[] BackRightPosition = BackRight.ModulePosition(GyroSensor.getAngle());
    double[] BackLeftPosition = BackLeft.ModulePosition(GyroSensor.getAngle());

    ModulePosition[0] = (FrontRightPosition[0] + 
    FrontLeftPosition[0] +
    BackLeftPosition[0] +
    BackRightPosition[0]) / 4;

    ModulePosition[1] = (FrontRightPosition[1] +
    FrontLeftPosition[1] +
    BackLeftPosition[1] +
    BackRightPosition[1]) / 4;

    ModulePosition[2] = (FrontRightPosition[2] +
    FrontLeftPosition[2] +
    BackLeftPosition[2] +
    BackRightPosition[2]) / 4;

    /*The rest of the function will calculate position based off the values from the odometry pods.
    If there is no Odometry pods attached to the robot you can just comment this portion out.*/
    double GyroAngle = Math.toRadians(GetGyroDegrees());

    CurrentOdometryPodX = Math.cos(GyroAngle) * VerticalOdometryEncoder.get() 
    + Math.sin(GyroAngle) * HorizontalOdometryEncoder.get();
    CurrentOdometryPodY = Math.sin(GyroAngle) * VerticalOdometryEncoder.get() 
    + Math.cos(GyroAngle) * HorizontalOdometryEncoder.get();
    
    OdometryPodPosition[0] += CurrentOdometryPodX - PastOdometryPodX;
    OdometryPodPosition[1] += CurrentOdometryPodY - PastOdometryPodY;

    PastOdometryPodX = CurrentOdometryPodX;
    PastOdometryPodY = CurrentOdometryPodY;

    SmartDashboard.putNumber("XPosistion", ModulePosition[0]);
    SmartDashboard.putNumber("YPosition", ModulePosition[1]);
    SmartDashboard.putNumber("Angle", GetGyroDegrees());
    SmartDashboard.putNumber("Velocity", ModulePosition[2]);
  }

  public double[] GetRobotPosition(){
    /*returns the position of the robot as {X position, Y position, and velocity}. ModuleFullPosition
    will return the position of the robot based on the values from the swerve modules. OdometryPodFullPosition
    will return the position based on the odometry pods. Just comment out which one you don't want to use.*/
    return(ModulePosition);
    //return(OdometryPodPosition);
  }

  public void ResetRobotPosition(){
    ModulePosition[0] = 0;
    ModulePosition[1] = 0;
    FrontRight.ResetModulePosition();
    BackRight.ResetModulePosition();
    BackLeft.ResetModulePosition();
    FrontLeft.ResetModulePosition();

    OdometryPodPosition[0] = 0;
    OdometryPodPosition[1] = 0;
  }

}
