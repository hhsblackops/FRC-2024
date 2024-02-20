package frc.robot.Commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MoveArmDown extends CommandBase{
    private final ShooterSubsystem shooterSubsystem;


    public MoveArmDown(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }


    @Override
    public void execute(){
        shooterSubsystem.MoveArmUp(-1);
    }

    @Override
    public boolean isFinished(){
        return(false);
    }
}
