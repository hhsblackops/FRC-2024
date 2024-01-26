/*This is the command that we call while driving the robot. We just get the values from the controller
and plug it straight into the Drive function in the DriveSubsystem.*/
package frc.robot.Commands.DriveCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;


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

    double[] Path = {};
    @Override
    public void initialize(){


        //PastPosition = driveSubsystem.GetRobotPosition();
    }

    double[] PastPosition = {0,0};
    @Override
    public void execute(){
        double realTimeLeftX = LeftXFunction.get();
        double realTimeLeftY = LeftYFunction.get();
        double realTimeRightX = RightXFunction.get();


        driveSubsystem.Drive(realTimeLeftX, realTimeLeftY, realTimeRightX);
    }

}