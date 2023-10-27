package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveDriveCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController;


public class RobotContainer {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private final XboxController DriveController = new XboxController(0);

    public RobotContainer(){
        driveSubsystem.setDefaultCommand(
            new SwerveDriveCommand(driveSubsystem, () -> DriveController.getLeftX(), () -> -DriveController.getLeftY(), () -> DriveController.getRightX())
        );
        configureButtonBinding();
    }

    private void configureButtonBinding(){
    }

    public Command getAutonomousCommand(){
        return new SequentialCommandGroup(
            new DriveCommand(driveSubsystem, 5, 5, 90)
        );

    }
}
