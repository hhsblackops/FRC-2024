package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroCommand extends CommandBase{


    private final DriveSubsystem driveSubsystem;

    public ResetGyroCommand(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){ 
        driveSubsystem.ResetGyro();
    }


  
    @Override
    public boolean isFinished(){
        return(true);
        
    }
}
