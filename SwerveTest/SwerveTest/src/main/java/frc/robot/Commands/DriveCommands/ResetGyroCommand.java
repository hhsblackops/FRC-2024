//This command just resets the gyro sensor so it's really easy to do it at the beginning of autonomous.
package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;

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
