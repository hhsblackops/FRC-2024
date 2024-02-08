package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class Shoot extends CommandBase{


    private final ShooterSubsystem shooterSubsystem;


    public Shoot(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize(){  
        shooterSubsystem.Shoot();
    }


  
    @Override
    public boolean isFinished(){
        return(false);
        
    }
}