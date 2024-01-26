package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootNeg extends CommandBase{


    private final ShooterSubsystem shooterSubsystem;


    public ShootNeg(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize(){  
        shooterSubsystem.TurnOnNeg();
    }


  
    @Override
    public boolean isFinished(){
        return(false);
        
    }
}