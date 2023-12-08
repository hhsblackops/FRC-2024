package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class GrabCommand extends CommandBase{
    
    private Timer Timer = new Timer();

    private final LiftSubsystem liftSubsystem;
    public GrabCommand(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize(){
        Timer.reset();
        Timer.start();
    }

    @Override
    public void execute(){    
        liftSubsystem.SetGrabber(0.2);
    }

    @Override
    public void end(boolean interrupted){
        liftSubsystem.SetGrabber(0.05);
    }

    @Override
    public boolean isFinished(){
        return((liftSubsystem.GetGrabberOutputLimit() > 10) && (Timer.get() > 0.2));
    }
}
