package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SlideForwardCommand;
import frc.robot.commands.SlideBackwardCommand;
import frc.robot.commands.StopSlideCommand;


import frc.robot.commands.SwerveDriveCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final LiftSubsystem liftSubsystem = new LiftSubsystem();
    private final SlideForwardCommand slideForward = new SlideForwardCommand(liftSubsystem);
    private final SlideBackwardCommand slideBackward = new SlideBackwardCommand(liftSubsystem);
    private final StopSlideCommand stopSlide = new StopSlideCommand(liftSubsystem);

    private final XboxController DriveController = new XboxController(0);
    private final JoystickButton AButton = new JoystickButton(DriveController, 1);
    private final JoystickButton BButton = new JoystickButton(DriveController, 2);
    public RobotContainer(){
        driveSubsystem.setDefaultCommand(
            new SwerveDriveCommand(driveSubsystem, () -> DriveController.getLeftX(), () -> -DriveController.getLeftY(), () -> DriveController.getRightX())
        );
        configureButtonBinding();
    }

    private void configureButtonBinding(){
        AButton.whileTrue(slideBackward);
        BButton.whileTrue(slideForward);
        AButton.whileFalse(stopSlide);
        BButton.whileFalse(stopSlide);
    }

    public Command getAutonomousCommand(){
        return new SequentialCommandGroup(

            new SlideBackwardCommand(liftSubsystem)
        );

    }
}
            //new DriveCommand(driveSubsystem, -5, 5, Math.PI)