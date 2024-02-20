package frc.robot.Commands.ShooterCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SensorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;


public class FixNote extends CommandBase{
    private final ShooterSubsystem shooterSubsystem;
    Timer Timer;

    public FixNote(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }


    @Override
    public void initialize(){
        shooterSubsystem.SetShooterIntake(0.1);
        Timer = new Timer();
        Timer.start();
    }

    @Override
    public void end(boolean interupted){
        shooterSubsystem.SetShooterIntake(0);
        shooterSubsystem.Shoot();
        shooterSubsystem.NoteHere();
    }

    @Override
    public boolean isFinished(){
        return(Timer.get() > 0.5);
    }
}
