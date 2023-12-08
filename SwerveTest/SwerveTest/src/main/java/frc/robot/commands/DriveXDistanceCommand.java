package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveXDistanceCommand extends CommandBase{


    private final DriveSubsystem driveSubsystem;
    private final double wantedXPosition;

    public DriveXDistanceCommand(DriveSubsystem driveSubsystem, double wantedXPosition){
        this.driveSubsystem = driveSubsystem;
        this.wantedXPosition = wantedXPosition;
        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize(){ 
        driveSubsystem.ResetRobotPosition();
    }

    
    @Override
    public void execute(){    
        if(wantedXPosition < driveSubsystem.GetRobotPosition()[0]){
            driveSubsystem.Drive(-0.3, 0, 0);
        }else{
            driveSubsystem.Drive(0.3, 0, 0);
        }

        SmartDashboard.putNumber("Wanted Position", wantedXPosition);
        SmartDashboard.putNumber("CurrentPosition", driveSubsystem.GetRobotPosition()[0]);
        SmartDashboard.putNumber("Difference", wantedXPosition - driveSubsystem.GetRobotPosition()[0]);
        SmartDashboard.putBoolean("Finished", (wantedXPosition - driveSubsystem.GetRobotPosition()[0]) < (6/12));
    
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.Drive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
        if(wantedXPosition < 0){
            return(wantedXPosition - driveSubsystem.GetRobotPosition()[0] > -(6/12));
        }else{
            return(wantedXPosition - driveSubsystem.GetRobotPosition()[0] < (6/12));
        }
    }
}
