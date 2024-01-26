package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterOff extends CommandBase{


    private final ShooterSubsystem shooterSubsystem;


    public ShooterOff(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){  
        shooterSubsystem.TurnOff();
    }


  
    @Override
    public boolean isFinished(){
        return(false);
        
    }
}