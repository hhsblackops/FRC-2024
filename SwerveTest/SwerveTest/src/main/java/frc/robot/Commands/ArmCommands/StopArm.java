package frc.robot.Commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class StopArm extends CommandBase{
    private final ShooterSubsystem shooterSubsystem;


    public StopArm(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }


    @Override
    public void execute(){
        shooterSubsystem.StopArm();
    }

    @Override
    public boolean isFinished(){
        return(true);
    }
}
