package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase{


    private final DriveSubsystem driveSubsystem;
    private final double wantedXPosition;
    private final double wantedYPosition;
    private final double finalAngle;


    public DriveCommand(DriveSubsystem driveSubsystem, double wantedXPosition, double wantedYPosition, double finalAngle){
        this.driveSubsystem = driveSubsystem;
        this.wantedYPosition = wantedYPosition;
        this.wantedXPosition = wantedXPosition;
        this.finalAngle = finalAngle;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){ 
        driveSubsystem.ResetRobotPosition();
        driveSubsystem.ResetGyro();
    }

    public double DriveMag = 0;

    @Override
    public void execute(){
        double[] RobotPosition = driveSubsystem.RobotPosition();
        //SmartDashboard.putNumber("X Position", RobotPosition[0]);
        //SmartDashboard.putNumber("Y Position", RobotPosition[1]);

        double RobotXPosition = RobotPosition[0];
        double RobotYPosition = RobotPosition[1];
        double XError = wantedXPosition - RobotXPosition;
        double YError = wantedYPosition - RobotYPosition;
        double DriveDirection = Math.atan2(XError, YError);
        DriveMag = Math.max(Math.hypot(XError, YError), 1);
        double x2;
        double TurnError = finalAngle - driveSubsystem.GetGyro();
        if(TurnError > 1){
            x2 = 0.5;
        }else if(TurnError < -1){
            x2 = -0.5;
        }else{
            x2 = 0;
        }
        
        driveSubsystem.Drive(DriveMag * Math.sin(DriveDirection), DriveMag * Math.cos(DriveDirection), x2);
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.Drive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return(DriveMag < 0.1);
        
    }
}
