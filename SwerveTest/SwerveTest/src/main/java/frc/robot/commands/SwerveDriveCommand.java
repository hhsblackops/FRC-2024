package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDriveCommand extends CommandBase{
    
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> LeftXFunction, LeftYFunction, RightXFunction;

    public SwerveDriveCommand(DriveSubsystem driveSubsystem,
    Supplier<Double> LeftXFunction, Supplier<Double> LeftYFunction, Supplier<Double> RightXFunction){
        this.driveSubsystem = driveSubsystem;
        this.LeftXFunction = LeftXFunction;
        this.LeftYFunction = LeftYFunction;
        this.RightXFunction = RightXFunction;
        addRequirements(driveSubsystem);
    }


    @Override
    public void execute(){
        double realTimeLeftX = LeftXFunction.get();
        double realTimeLeftY = LeftYFunction.get();
        double realTimeRightX = RightXFunction.get();
        SmartDashboard.putNumber("XPosistion", driveSubsystem.GetRobotPosition()[0]);
        SmartDashboard.putNumber("YPosition", driveSubsystem.GetRobotPosition()[1]);
        SmartDashboard.putNumber("Angle", driveSubsystem.GetGyroDegrees());
        SmartDashboard.putNumber("Velocity", driveSubsystem.GetRobotPosition()[2]);

        driveSubsystem.Drive(realTimeLeftX, realTimeLeftY, realTimeRightX);

    }
}