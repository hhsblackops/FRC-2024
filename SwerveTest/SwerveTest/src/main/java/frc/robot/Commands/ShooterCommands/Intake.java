package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SensorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;



public class Intake extends CommandBase{
    private final ShooterSubsystem shooterSubsystem;
    private final SensorSubsystem sensorSubsystem;


    public Intake(ShooterSubsystem shooterSubsystem, SensorSubsystem sensorSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        addRequirements(shooterSubsystem);

    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("blue", sensorSubsystem.GetColor().blue);
        SmartDashboard.putNumber("red", sensorSubsystem.GetColor().red);
        SmartDashboard.putNumber("green", sensorSubsystem.GetColor().green);
        shooterSubsystem.SetShooterIntake(-0.5);
        shooterSubsystem.SetIntake(0.5);
        shooterSubsystem.SetArmPosition(0);
    }

    @Override
    public void end(boolean interupted){
        shooterSubsystem.SetIntake(0);
        shooterSubsystem.SetShooterIntake(0);
        shooterSubsystem.NoteHere();
    }

    @Override
    public boolean isFinished(){
        return((sensorSubsystem.GetColor().red > 0.40) || (sensorSubsystem.GetColor().blue < 0.19));
    }
}
