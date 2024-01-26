package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootPos extends CommandBase{


    private final ShooterSubsystem shooterSubsystem;


    public ShootPos(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize(){  
        shooterSubsystem.TurnOnPos();
    }


  
    @Override
    public boolean isFinished(){
        return(false);
        
    }
}