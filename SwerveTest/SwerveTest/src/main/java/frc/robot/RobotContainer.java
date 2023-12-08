package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;

import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.DriveXDistanceCommand;
import frc.robot.commands.SlideCommand;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.SpitCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.RestArmCommand;


import frc.robot.commands.SwerveDriveCommand;

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
    private final LiftSubsystem liftSubsystem = new LiftSubsystem();

    private final SlideCommand slideLeft = new SlideCommand(liftSubsystem, 1);
    private final SlideCommand slideRight = new SlideCommand(liftSubsystem, -1);
    private final SlideCommand slideIn = new SlideCommand(liftSubsystem, 0);

    private final SpitCommand spit = new SpitCommand(liftSubsystem);
    private final GrabCommand grab = new GrabCommand(liftSubsystem);

    private final ArmCommand moveArmLeft = new ArmCommand(liftSubsystem, 105);
    private final ArmCommand moveArmRight = new ArmCommand(liftSubsystem, -105);
    private final ArmCommand resetArm = new ArmCommand(liftSubsystem, 0);


    private final Command getOverItLeft = new SequentialCommandGroup(
        new ArmCommand(liftSubsystem, 105),
        new SlideCommand(liftSubsystem, 1),
        new SlideCommand(liftSubsystem, 1),
        new WaitCommand(1.7),
        new DriveXDistanceCommand(driveSubsystem, -1.35),
        new ArmCommand(liftSubsystem, -105),
        new SlideCommand(liftSubsystem, -1),
        new SlideCommand(liftSubsystem, -1),
        new WaitCommand(2),
        new DriveXDistanceCommand(driveSubsystem, -1.3),
        new ArmCommand(liftSubsystem, -45),
        new WaitCommand(1),
        new SlideCommand(liftSubsystem, 0),
        new DriveXDistanceCommand(driveSubsystem, -1.5),
        new RestArmCommand(liftSubsystem)

    );

    private final Command getOverItRight = new SequentialCommandGroup(
        new ArmCommand(liftSubsystem, 105),
        new SlideCommand(liftSubsystem, 1),
        new SlideCommand(liftSubsystem, 1),
        new WaitCommand(1.7),
        new DriveXDistanceCommand(driveSubsystem, 1.35),        
        new ArmCommand(liftSubsystem, -105),
        new SlideCommand(liftSubsystem, -1),
        new SlideCommand(liftSubsystem, -1),
        new WaitCommand(2),
        new DriveXDistanceCommand(driveSubsystem, 1.3),
        new ArmCommand(liftSubsystem, 45),
        new WaitCommand(0.5),
        new SlideCommand(liftSubsystem, 0)
    );



    private final XboxController DriveController = new XboxController(0);
    private final JoystickButton XButton = new JoystickButton(DriveController, 3);
    private final JoystickButton YButton = new JoystickButton(DriveController, 4);
    private final JoystickButton BButton = new JoystickButton(DriveController, 2);
    private final JoystickButton AButton = new JoystickButton(DriveController, 1);

    private final JoystickButton RightBumper = new JoystickButton(DriveController, 6);
    private final JoystickButton LeftBumper = new JoystickButton(DriveController, 5);

    private final XboxController LiftController = new XboxController(1);

    private final JoystickButton LiftRightBumper = new JoystickButton(LiftController, 6);
    private final JoystickButton LiftLeftBumper = new JoystickButton(LiftController, 5);

    private final JoystickButton LiftAButton = new JoystickButton(LiftController, 1);
    private final JoystickButton LiftBButton = new JoystickButton(LiftController, 2);
    private final JoystickButton LiftXButton = new JoystickButton(LiftController, 3);
    private final JoystickButton LiftYButton = new JoystickButton(LiftController, 4);

    private final POVButton LeftDPad = new POVButton(DriveController, -90);
    private final POVButton RightDPad = new POVButton(DriveController, 90);
    
    public RobotContainer(){
        driveSubsystem.setDefaultCommand(
            new SwerveDriveCommand(
                driveSubsystem,
                () -> (-DriveController.getLeftY() * (1 - (0.5 * DriveController.getLeftTriggerAxis()))),
                () -> (-DriveController.getLeftX() * (1 - (0.5 * DriveController.getLeftTriggerAxis()))),
                () -> (DriveController.getRightX() * (1 - (0.5 * DriveController.getLeftTriggerAxis())))
            )
            
        );
        configureButtonBinding();
    }

    private void configureButtonBinding(){
        //LeftBumper.onTrue(getOverItLeft);
        //RightBumper.onTrue(getOverItRight);
        //AButton.onTrue(grab);
        //BButton.onTrue(spit);

        LiftLeftBumper.onTrue(getOverItLeft);
        LiftRightBumper.onTrue(getOverItRight);
        //LiftLeftBumper.onTrue(slideLeft);
        //LiftRightBumper.onTrue(slideRight);
        
        LiftAButton.onTrue(grab);
        LiftBButton.onTrue(
            new SequentialCommandGroup(
                spit,
                new ArmCommand(liftSubsystem, 0)
            )
        );
        LiftXButton.onTrue(
            new SequentialCommandGroup(
                new ArmCommand(liftSubsystem, -45),
                new WaitCommand(0.5),
                new RestArmCommand(liftSubsystem)
            )
        );

        LiftYButton.onTrue(new ArmCommand(liftSubsystem, 25));


        

    }

    public Command getAutonomousCommand(){
        return(
            new SequentialCommandGroup(
                new ResetGyroCommand(driveSubsystem),
                new ArmCommand(liftSubsystem, 105),
                new SlideCommand(liftSubsystem, 1),
                new SlideCommand(liftSubsystem, 1.1),
                new WaitCommand(1.7),
                new DriveXDistanceCommand(driveSubsystem, -1.35),
                new ArmCommand(liftSubsystem, -105),
                new SlideCommand(liftSubsystem, -1),
                new SlideCommand(liftSubsystem, -1),
                new WaitCommand(2),
                new DriveXDistanceCommand(driveSubsystem, -1.3),
                new ArmCommand(liftSubsystem, -45),
                new WaitCommand(1),
                new SlideCommand(liftSubsystem, 0),
                new DriveXDistanceCommand(driveSubsystem, -1.5),
                new RestArmCommand(liftSubsystem)
            )
        );
    }
}