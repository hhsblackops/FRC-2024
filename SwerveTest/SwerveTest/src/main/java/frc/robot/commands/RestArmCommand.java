package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class RestArmCommand extends CommandBase{


    private final LiftSubsystem liftSubsystem;

    public RestArmCommand(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize(){ 
        liftSubsystem.SetCoast();
    }

    @Override
    public boolean isFinished(){
        return(true);
    }
}