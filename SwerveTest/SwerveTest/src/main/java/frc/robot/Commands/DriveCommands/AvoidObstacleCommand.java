/*This command is meant for moving from point A to point B but there is an obstacle in the way. To get around
the obstacle, you put the coordinates of the of where it is and the radius that you want the center of the 
robot to stay away from, and it will make the robot automatically go to the tangent line of the circle
around the obstacle, follow that circle until it sees that it will leave to where you want it to finish the
program at. To make it go to the right around the circle (counterclockwise) put in +1 as the parameter
called ObstacleSide and if you want to go to the left of around the circle (clockwise) then put in -1.
*/
package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class AvoidObstacleCommand extends CommandBase{


    private final DriveSubsystem driveSubsystem;
    private final double WantedXPosition;
    private final double WantedYPosition;
    private final double WantedAngle;
    private final double ObstacleRadius;
    private final double[] ObstaclePosition;
    private final double ObstacleSide;


    public AvoidObstacleCommand(DriveSubsystem driveSubsystem, double WantedXPosition, double WantedYPosition, double WantedAngle, double ObstacleRadius, double[] ObstaclePosition, double ObstacleSide){
        this.driveSubsystem = driveSubsystem;
        this.WantedYPosition = WantedYPosition;
        this.WantedXPosition = WantedXPosition;
        this.WantedAngle = WantedAngle;
        this.ObstacleRadius = ObstacleRadius;
        this.ObstaclePosition = ObstaclePosition;
        this.ObstacleSide = ObstacleSide;
        addRequirements(driveSubsystem);
        
    }
    double ObstacleCircum;
    double ObstacleXPosition;
    double ObstacleYPosition;
    double step;
    PIDController StrafePIDController;
    PIDController RotatePIDController;
    

    @Override
    public void initialize(){ 
        driveSubsystem.ResetRobotPosition();
        driveSubsystem.ResetGyro();
        ObstacleCircum = 2 * Math.PI * ObstacleRadius;
        ObstacleXPosition = ObstaclePosition[0];
        ObstacleYPosition = ObstaclePosition[1];
        
        StrafePIDController = new PIDController(DriveConstants.StrafeP, DriveConstants.StrafeI, DriveConstants.StrafeD);
        StrafePIDController.setSetpoint(0);

        RotatePIDController = new PIDController(DriveConstants.RotateP, DriveConstants.RotateI, DriveConstants.RotateD);
        RotatePIDController.setSetpoint(Math.IEEEremainder(WantedAngle, 2 * Math.PI));
        RotatePIDController.enableContinuousInput(-Math.PI, Math.PI);
        step = 1;

    }

    double DriveMagnitude;
    double RobotXPosition;
    double RobotYPosition;
    double CurrentAngle;
    double DriveDirection;

    
    
    @Override
    public void execute(){    
        double[] RobotPosition = driveSubsystem.GetRobotPosition();
        RobotXPosition = RobotPosition[0];
        RobotYPosition = RobotPosition[1];
        double RobotVelocity = RobotPosition[2];

        /*This next variable is the magnitude for the vector of what we're setting our robot to move
        to. This is just calculated based off of the difference between where we are and where we 
        want to go. We throw this into the PID controller and it will give us the magnitude we want.*/
        DriveMagnitude = Math.abs(StrafePIDController.calculate(Math.hypot((WantedXPosition - RobotXPosition), (WantedYPosition - RobotYPosition))));
        
        if(step == 1){
            /*To get to the circle of the object and then follow the circle we just set the robot to 
            follow the tangent line of the cirlce. We can find this by representing the position of the robot,
            the point of the obstacle, and the point which the tangent line touches the circle around the 
            obstacle as a right triangle.
             */
            double RobotToObstacleDist = Math.hypot(ObstacleXPosition - RobotXPosition, ObstacleYPosition - RobotYPosition);
            /*This next angle is what angle the tangent line will be, which is what we'll just set our
            Direction to.*/
            double TangentLineAngle = (Math.asin(ObstacleRadius / RobotToObstacleDist) * ObstacleSide) + Math.atan2(ObstacleXPosition - RobotXPosition, ObstacleYPosition - RobotYPosition);
            DriveDirection = Math.IEEEremainder(TangentLineAngle, 2 * Math.PI);
        
            /*These next 2 variables are used to see when we need to cut out of the the tangent line of the
            circle and start driving directly to the position we want to end in.*/
            double ObstacleToWantedDist = Math.hypot(ObstaclePosition[0] - WantedXPosition, ObstaclePosition[1] - WantedYPosition);
            double TangentToWanted = Math.IEEEremainder((Math.atan2(ObstacleXPosition - WantedXPosition, ObstacleYPosition - WantedYPosition) + (Math.asin(ObstacleRadius / ObstacleToWantedDist) * -ObstacleSide)) + Math.PI, 2 * Math.PI);
            
            if((TangentToWanted < DriveDirection + 0.1) && (TangentToWanted > DriveDirection - 0.1)){
                step += 1;
            }
        }else{
            DriveDirection = Math.atan2((WantedXPosition - RobotXPosition), (WantedYPosition - RobotYPosition));
        }

        CurrentAngle = Math.IEEEremainder(Math.toRadians(driveSubsystem.GetGyroDegrees()), 2 * Math.PI);
        
        driveSubsystem.Drive(DriveMagnitude * Math.sin(DriveDirection), DriveMagnitude * Math.cos(DriveDirection), RotatePIDController.calculate(CurrentAngle));
        SmartDashboard.putNumber("Step", step);
        SmartDashboard.putNumber("XPosistion", RobotXPosition);
        SmartDashboard.putNumber("YPosition", RobotYPosition);
        SmartDashboard.putNumber("Angle", driveSubsystem.GetGyroDegrees());
        SmartDashboard.putNumber("Velocity", RobotVelocity);
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.Drive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return((Math.hypot((WantedXPosition - RobotXPosition), (WantedYPosition - RobotYPosition)) < 0.01) && (Math.abs(CurrentAngle - WantedAngle) < 0.01));
    }
}
