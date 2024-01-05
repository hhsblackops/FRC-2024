package frc.robot.Commands.DriveCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class FollowPath extends CommandBase{


    private final double[] PathList;
    private final DriveSubsystem driveSubsystem;


    public FollowPath(DriveSubsystem driveSubsystem, double[] PathList){
        this.PathList = PathList;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    PIDController StrafePIDController;
    PIDController RotatePIDController;
    int step;
    double DriveDirection;
    double NextXPosition;
    double NextYPosition;
    double DirectionToNext;
    double RobotXPosition;
    double RobotYPosition;

    @Override 
    public void initialize(){  
        driveSubsystem.Drive(0,0,PathList[2]);
        driveSubsystem.ResetRobotPosition();

        step = 0;
        RotatePIDController = new PIDController(DriveConstants.RotateP, DriveConstants.RotateI, DriveConstants.RotateD);
        RotatePIDController.setSetpoint(Math.IEEEremainder(0, 2 * Math.PI));
        RotatePIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        StrafePIDController = new PIDController(DriveConstants.StrafeP, DriveConstants.StrafeI, DriveConstants.StrafeD);
        StrafePIDController.setSetpoint(0);
    }

    int Pretend = 0;
    boolean finished = false;
    @Override
    public void execute(){
        RobotXPosition = driveSubsystem.GetRobotPosition()[0];
        RobotYPosition = driveSubsystem.GetRobotPosition()[1];

        double DriveMagnitude = Math.abs(StrafePIDController.calculate(Math.hypot((PathList[PathList.length - 3] - RobotXPosition), (PathList[PathList.length - 2] - RobotYPosition))));
        //double DriveMagnitude = 1;
        boolean XCondition = true;
        boolean YCondition = true;

        while((XCondition || YCondition) && (step < PathList.length - 4)){
            NextXPosition = PathList[step];
            NextYPosition = PathList[step + 1];
            DirectionToNext = PathList[step + 2];
            if(DirectionToNext > 0){
                /*We see what direction we're driving in, which the drive direction will always be on the 
                interval -pi to pi since it will come out of the Math.atan2 function, and if it's positive
                (driving right) we want to check if the X value is going to be greater.*/
                if(RobotXPosition < NextXPosition){
                    XCondition = false;
                }
            }else if((DirectionToNext == 0) || (DirectionToNext == Math.PI) ||(DirectionToNext == -Math.PI)){
                XCondition = false;
            }else{
                //If we're driving to the left then we'll see if it is going to be less then it.
                if(RobotXPosition > NextXPosition){
                    XCondition = false;
                }
            }
            if((DirectionToNext > Math.PI / -2) && (DirectionToNext < Math.PI / 2)){
                /*This is the same as we do for the X direction, but now we check to see if the Y value
                is going forward or backward.*/
                if(RobotYPosition < NextYPosition){
                    YCondition = false;
                }
            }else if((DriveDirection == Math.PI / -2) || (DirectionToNext < Math.PI / 2)){
                YCondition = false;
            }else{
                if(RobotYPosition > NextYPosition){
                    YCondition = false;
                }
            }
            if((XCondition || YCondition) && (step < PathList.length - 4)){
                step += 3;
                System.out.println("Stucker");

            }
        }
        finished = (step > PathList.length - 4);

        DriveDirection = (Math.atan2(NextXPosition - RobotXPosition, NextYPosition - RobotYPosition) * 0.5) + (DirectionToNext * 0.5);
        //DriveDirection = DirectionToNext;
        driveSubsystem.Drive((DriveMagnitude * Math.sin(DriveDirection)), (DriveMagnitude * Math.cos(DriveDirection)), RotatePIDController.calculate(Math.toRadians(driveSubsystem.GetGyroDegrees())));//RotatePIDController.calculate(CurrentAngle)
        //driveSubsystem.Drive(Math.sin(PathList[Pretend + 2]), Math.cos(PathList[Pretend + 2]), 0);
        //Pretend += 3;
    }


    @Override
    public void end(boolean interrupted){
        driveSubsystem.Drive(0,0,0);
    }
  
    @Override
    public boolean isFinished(){
        //return((Math.hypot((PathList[PathList.length - 3] - RobotXPosition), (PathList[PathList.length - 2] - RobotYPosition)) < 0.01));// && (Math.abs(CurrentAngle - WantedAngle) < 0.01));
        return(finished);
    }
}