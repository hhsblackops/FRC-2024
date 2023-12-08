package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.Timer;




public class SpitCommand extends CommandBase{
    
    private final LiftSubsystem liftSubsystem;
    private Timer Timer = new Timer();

    public SpitCommand(LiftSubsystem liftSubsystem){
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
        liftSubsystem.SetGrabber(-0.05);
    }

    @Override
    public void end(boolean interrupted){
        liftSubsystem.SetGrabber(0);
        liftSubsystem.SetArmPosition(0);
    }

    @Override
    public boolean isFinished(){
        return(Timer.get() > 0.5);
    }
}
