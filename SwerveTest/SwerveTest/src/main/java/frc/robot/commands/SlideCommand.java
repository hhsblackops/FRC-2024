package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

public class SlideCommand extends CommandBase{
    private Timer Timer = new Timer();


    private final LiftSubsystem liftSubsystem;
    private final double wantedPosition;

    public SlideCommand(LiftSubsystem liftSubsystem, double wantedPosition){
        this.liftSubsystem = liftSubsystem;
        this.wantedPosition = wantedPosition;

        addRequirements(liftSubsystem);
    }

    
    @Override
    public void initialize(){ 
        liftSubsystem.SetSlidePID(0.2, 0, 0);
        liftSubsystem.SetSlideInitial();
        liftSubsystem.SetSlidePosition(wantedPosition);
        Timer.reset();
        Timer.start();


    }
    
    @Override
    public void end(boolean isFinished){   
        liftSubsystem.StopSlide();
        liftSubsystem.SaveSlideSide(wantedPosition);
    }

    @Override
    public boolean isFinished(){
        return(((liftSubsystem.GetSlideOutputLimit() > 11) || ((wantedPosition == 0) && (liftSubsystem.GetSlideAppliedOutput() < 0.1) && (liftSubsystem.GetSlideAppliedOutput() > -0.1))) && (Timer.get() > 0.5));
    }
}
