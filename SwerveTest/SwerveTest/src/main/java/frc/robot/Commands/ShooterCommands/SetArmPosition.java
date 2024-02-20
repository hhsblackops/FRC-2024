package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SetArmPosition extends CommandBase{


    private final ShooterSubsystem shooterSubsystem;
    private final double position;


    public SetArmPosition(ShooterSubsystem shooterSubsystem, double position){
        this.shooterSubsystem = shooterSubsystem;
        this.position = position;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize(){  
        shooterSubsystem.SetArmPosition(Math.toRadians(position));
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("ArmPosition", -Math.toDegrees(shooterSubsystem.GetArmPosition()));
    }

  
    @Override
    public boolean isFinished(){
        return(false);
        
    }
}