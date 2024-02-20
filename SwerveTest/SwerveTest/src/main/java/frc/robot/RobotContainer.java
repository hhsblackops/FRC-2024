package frc.robot;

import frc.robot.Commands.DriveCommands.AvoidObstacleCommand;

import frc.robot.Commands.DriveCommands.DriveCommand;
import frc.robot.Commands.DriveCommands.SwerveDriveCommand;

import frc.robot.Commands.OtherCommands.LEDCommand;

import frc.robot.Commands.ShooterCommands.Shoot;
import frc.robot.Commands.ShooterCommands.ShooterOff;
import frc.robot.Commands.ShooterCommands.SetArmPosition;
import frc.robot.Commands.ShooterCommands.FixNote;
import frc.robot.Commands.ShooterCommands.Intake;
import frc.robot.Commands.ArmCommands.MoveArmDown;
import frc.robot.Commands.ArmCommands.MoveArmUp;
import frc.robot.Commands.ArmCommands.StopArm;




import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.SensorSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final SensorSubsystem sensorSubsystem = new SensorSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final Command shoot = new Shoot(shooterSubsystem);
    private final Command shooterOff = new ShooterOff(shooterSubsystem);
    private final Command intake = new Intake(shooterSubsystem, sensorSubsystem);
    private final Command fixNote = new FixNote(shooterSubsystem);

    
 
    private final XboxController DriveController = new XboxController(0);
    private final JoystickButton XButton = new JoystickButton(DriveController, 3);
    private final JoystickButton YButton = new JoystickButton(DriveController, 4);
    private final JoystickButton BButton = new JoystickButton(DriveController, 2);
    private final JoystickButton AButton = new JoystickButton(DriveController, 1);

    private final JoystickButton RightBumper = new JoystickButton(DriveController, 6);
    private final JoystickButton LeftBumper = new JoystickButton(DriveController, 5);

    private final Joystick ButtonBoard = new Joystick(1);
    private final JoystickButton ShootButton = new JoystickButton(ButtonBoard, 1);
    
    public RobotContainer(){
        driveSubsystem.setDefaultCommand(
            new SwerveDriveCommand(
                driveSubsystem, shooterSubsystem, sensorSubsystem,
                () -> (DriveController.getLeftX() * (1 - (0.5 * DriveController.getLeftTriggerAxis()))),
                () -> (-DriveController.getLeftY() * (1 - (0.5 * DriveController.getLeftTriggerAxis()))),
                () -> ((DriveController.getRightX() * 0.5) * (1 - (0.5 * DriveController.getLeftTriggerAxis())))
            )
            
        );
        configureButtonBinding();
    }

    private void configureButtonBinding(){
        AButton.onTrue(shoot);
        XButton.onTrue(new SequentialCommandGroup(intake, fixNote)); 
        YButton.onTrue(new MoveArmUp(shooterSubsystem));
        YButton.onFalse(new StopArm(shooterSubsystem));
        BButton.onTrue(new MoveArmDown(shooterSubsystem));
        BButton.onFalse(new StopArm(shooterSubsystem));


        //YButton.onTrue(new SetArmPosition(shooterSubsystem, 0));
        //AButton.onTrue(new SetArmPosition(shooterSubsystem, 45));


    }
    double turn = 0;

    public Command getAutonomousCommand(){
        //return(new FollowPath2(driveSubsystem, Path));
        //return(new AvoidObstacleCommand(driveSubsystem, 4, 0, Math.toRadians(0), 1, Obstacle, 1));
        //return(new DriveCommand(driveSubsystem, 1, 1, Math.toRadians(45)));
        //return(new LEDCommand(ledSubsystem, 0.59));
        return(new SequentialCommandGroup(
            new DriveCommand(driveSubsystem, 0, 10, Math.toRadians(0)),
            new DriveCommand(driveSubsystem, -7.1, 7.1, Math.toRadians(90)),
            new DriveCommand(driveSubsystem, -10, 0, Math.toRadians(180)),
            new DriveCommand(driveSubsystem, -7.1, -7.1, Math.toRadians(270)),
            new DriveCommand(driveSubsystem, 0, -10, Math.toRadians(360)),
            new DriveCommand(driveSubsystem, 7.1, -7.1, Math.toRadians(90)),
            new DriveCommand(driveSubsystem, 10, 0, Math.toRadians(180)),
            new DriveCommand(driveSubsystem, 7.1, 7.1, Math.toRadians(270)),
            new DriveCommand(driveSubsystem, 0, 10, Math.toRadians(360)),
            new DriveCommand(driveSubsystem, 0, 0, Math.toRadians(0))));
            
    }
}