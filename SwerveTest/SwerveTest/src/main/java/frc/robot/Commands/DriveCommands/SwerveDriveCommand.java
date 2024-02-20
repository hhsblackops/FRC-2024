/*This is the command that we call while driving the robot. We just get the values from the controller
and plug it straight into the Drive function in the DriveSubsystem.*/
package frc.robot.Commands.DriveCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SensorSubsystem;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.controller.PIDController;


public class SwerveDriveCommand extends CommandBase{
    
    private final DriveSubsystem driveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final SensorSubsystem sensorSubsystem;

    private final Supplier<Double> LeftXFunction, LeftYFunction, RightXFunction;

    public SwerveDriveCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, SensorSubsystem sensorSubsystem,
    Supplier<Double> LeftXFunction, Supplier<Double> LeftYFunction, Supplier<Double> RightXFunction){
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.LeftXFunction = LeftXFunction;
        this.LeftYFunction = LeftYFunction;
        this.RightXFunction = RightXFunction;
        addRequirements(driveSubsystem);
    }

    PIDController CameraRotatePIDController;

    @Override
    public void initialize(){
        CameraRotatePIDController = new PIDController(0.01, 0, 0);

    }

    double realTimeStrafetX;
    double realTimeStrafeY;
    double realTimeRotateX;
    @Override
    public void execute(){
        realTimeStrafetX = LeftXFunction.get();
        realTimeStrafeY = LeftYFunction.get();

        if(shooterSubsystem.IsNote() && sensorSubsystem.IsTargetVisible()){
            CameraRotatePIDController.setSetpoint(sensorSubsystem.GetHorizontalOffset());
            realTimeRotateX = CameraRotatePIDController.calculate(0);
            double tv = sensorSubsystem.GetVerticalOffset();
            shooterSubsystem.SetArmPosition((-0.02 * tv) + 0.73);
        }else{
            realTimeRotateX = RightXFunction.get();
        }
        SmartDashboard.putNumber("PIDvalues", CameraRotatePIDController.calculate(0));

        driveSubsystem.Drive(realTimeStrafetX, realTimeStrafeY, realTimeRotateX);
    }
}