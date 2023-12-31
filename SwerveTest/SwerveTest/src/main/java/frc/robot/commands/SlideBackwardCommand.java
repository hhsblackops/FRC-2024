package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SlideBackwardCommand extends CommandBase{


    private final LiftSubsystem liftSubsystem;


    public SlideBackwardCommand(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize(){ 
    }


    @Override
    public void execute(){    
        liftSubsystem.SlideBackward();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return(false);
        
    }
}
