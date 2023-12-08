package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;



public class ArmCommand extends CommandBase{


    private final LiftSubsystem liftSubsystem;
    private final double wantedPosition;

    public ArmCommand(LiftSubsystem liftSubsystem, double wantedPosition){
        this.liftSubsystem = liftSubsystem;
        this.wantedPosition = wantedPosition;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize(){ 
        liftSubsystem.SetArmPID(0.1, 0, 0);
        liftSubsystem.SetArmPosition(wantedPosition);
    }

    @Override
    public void execute(){    
    }

    @Override
    public boolean isFinished(){
        return(true);
    }
}
