package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends CommandBase{


    private final DriveSubsystem driveSubsystem;
    private final double wantedXPosition;
    private final double wantedYPosition;
    private final double wantedAngle;
    PIDController DrivePIDController = new PIDController(1, 0, 0);
    PIDController TurnPIDController = new PIDController(0.5, 0, 0);


    public DriveCommand(DriveSubsystem driveSubsystem, double wantedXPosition, double wantedYPosition, double wantedAngle){
        this.driveSubsystem = driveSubsystem;
        this.wantedYPosition = wantedYPosition;
        this.wantedXPosition = wantedXPosition;
        this.wantedAngle = wantedAngle;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){ 
        driveSubsystem.ResetRobotPosition();
        driveSubsystem.ResetGyro();
        DrivePIDController.setSetpoint(0);
        TurnPIDController.setSetpoint(Math.IEEEremainder(wantedAngle, 2 * Math.PI));
        TurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }


    @Override
    public void execute(){    
        double[] RobotPosition = driveSubsystem.GetRobotPosition();
        double RobotXPosition = RobotPosition[0];
        double RobotYPosition = RobotPosition[1];
        double DriveMagnitude = Math.abs(DrivePIDController.calculate(Math.hypot((wantedXPosition - RobotXPosition), (wantedYPosition - RobotYPosition))));
        double DriveDirection = Math.atan2((wantedXPosition - RobotXPosition), (wantedYPosition - RobotYPosition));
        SmartDashboard.putNumber("XPosistion", RobotXPosition);
        SmartDashboard.putNumber("YPosition", RobotYPosition);
        double CurrentAngle = Math.IEEEremainder(Math.toRadians(driveSubsystem.GetGyroDegrees()), 2 * Math.PI);

        SmartDashboard.putNumber("Angle", Math.toDegrees(CurrentAngle));
        driveSubsystem.Drive((DriveMagnitude * Math.sin(DriveDirection)), (DriveMagnitude * Math.cos(DriveDirection)), TurnPIDController.calculate(CurrentAngle));
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.Drive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return(false);
        
    }
}
