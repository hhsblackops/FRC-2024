//This command is one that will drive straight to a set of coordinates that you put in.
package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem;


public class DriveCommand extends CommandBase{


    private final DriveSubsystem driveSubsystem;
    private final double WantedXPosition;
    private final double WantedYPosition;
    private final double WantedAngle;



    public DriveCommand(DriveSubsystem driveSubsystem, double WantedXPosition, double WantedYPosition, double WantedAngle){
        this.driveSubsystem = driveSubsystem;
        this.WantedYPosition = WantedYPosition;
        this.WantedXPosition = WantedXPosition;
        this.WantedAngle = WantedAngle;
        addRequirements(driveSubsystem);
    }

    PIDController StrafePIDController; 
    PIDController RotatePIDController;

    @Override
    public void initialize(){ 
        driveSubsystem.ResetRobotPosition();
        driveSubsystem.ResetGyro();

        StrafePIDController = new PIDController(DriveConstants.StrafeP, DriveConstants.StrafeI, DriveConstants.StrafeD);
        StrafePIDController.setSetpoint(0);

        RotatePIDController = new PIDController(DriveConstants.RotateP, DriveConstants.RotateI, DriveConstants.RotateD);
        RotatePIDController.setSetpoint(Math.IEEEremainder(WantedAngle, 2 * Math.PI));
        RotatePIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    double DriveMagnitude;
    double RobotXPosition;
    double RobotYPosition;
    double CurrentAngle;

    @Override
    public void execute(){    
        //First we get the position of the robot
        double[] RobotPosition = driveSubsystem.GetRobotPosition();
        RobotXPosition = RobotPosition[0];
        RobotYPosition = RobotPosition[1];
        /*this calulates the input for strafeing by getting our differency in X and Y posision fron where
        we are compared to where we want to go and then turns it into a vector with the same direction
        but the magnitude is changed by running it through the PID controller.*/
        DriveMagnitude = Math.abs(StrafePIDController.calculate(Math.hypot((WantedXPosition - RobotXPosition), (WantedYPosition - RobotYPosition))));
        double DriveDirection = Math.atan2((WantedXPosition - RobotXPosition), (WantedYPosition - RobotYPosition));

        CurrentAngle = Math.IEEEremainder(Math.toRadians(driveSubsystem.GetGyroDegrees()), 2 * Math.PI);

        /*We then break this back down into X and Y components to drive it using the Drive Function.
        We calculate the how much we want to rotate the robot using the PID controller and pluging in
        what direction the robot is currently facing.*/
        //driveSubsystem.Drive((DriveMagnitude * Math.sin(DriveDirection)), (DriveMagnitude * Math.cos(DriveDirection)), RotatePIDController.calculate(CurrentAngle));
        driveSubsystem.Drive(0.5, 0.5, 0);
        SmartDashboard.putNumber("XPosistion", driveSubsystem.GetRobotPosition()[0]);
        SmartDashboard.putNumber("YPosition", driveSubsystem.GetRobotPosition()[1]);
        SmartDashboard.putNumber("Angle", driveSubsystem.GetGyroDegrees());
        SmartDashboard.putNumber("Velocity", driveSubsystem.GetRobotPosition()[2]);
    }
    

    @Override
    public void end(boolean interrupted){
        driveSubsystem.Drive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return((Math.hypot((WantedXPosition - RobotXPosition), (WantedYPosition - RobotYPosition)) < 0.01) && (Math.abs(CurrentAngle - WantedAngle) < 0.01));
        
    }
}
