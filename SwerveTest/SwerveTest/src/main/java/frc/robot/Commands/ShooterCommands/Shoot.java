package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SensorSubsystem;

import edu.wpi.first.wpilibj.Timer;


public class Shoot extends CommandBase{


    private final ShooterSubsystem shooterSubsystem;
    Timer Timer = new Timer();

    public Shoot(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize(){
        shooterSubsystem.Shoot();
        shooterSubsystem.SetShooterIntake(-0.5);
        Timer.reset();
        Timer.start();
    }

    @Override
    public void end(boolean interupted){
        shooterSubsystem.ShooterOff();
        shooterSubsystem.SetShooterIntake(0);
        shooterSubsystem.NoteGone();
    }
  
    @Override
    public boolean isFinished(){
        return(Timer.get() > 1);
    }
}