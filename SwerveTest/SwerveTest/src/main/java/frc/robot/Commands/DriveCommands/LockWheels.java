package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;


public class LockWheels extends CommandBase{


    private final DriveSubsystem driveSubsystem;


    public LockWheels(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.Lock();
    }
}

