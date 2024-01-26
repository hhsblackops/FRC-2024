package frc.robot.Modules;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;


public class SwerveModule{
    
    private CANSparkMax MovingSpark;
    private SparkMaxPIDController MovingPIDController;

    private CANSparkMax TurningSpark;
    private SparkMaxPIDController TurningPIDController;
    private AbsoluteEncoder TurningEncoder;
    private RelativeEncoder MovingEncoder;
    private double SetPosition;
    private double AngleOffset;

    public SwerveModule(int TurningID, int MovingID, double Offset){
        /*The 2 different motors in each module is the turning (changes the direction of the wheel)
        and moving (drives the wheel)*/

        MovingSpark = new CANSparkMax(MovingID, MotorType.kBrushless);
        MovingSpark.restoreFactoryDefaults();
        MovingSpark.setIdleMode(IdleMode.kBrake);
        MovingSpark.setSmartCurrentLimit(40);

        MovingEncoder = MovingSpark.getEncoder();
        MovingEncoder.setPositionConversionFactor(DriveConstants.DrivingPositionFactor);
        MovingEncoder.setVelocityConversionFactor(DriveConstants.DrivingVelocityFactor);
        MovingEncoder.setPosition(0);

        MovingPIDController = MovingSpark.getPIDController();
        MovingPIDController.setFeedbackDevice(MovingEncoder);

        //PID values for the moving wheel
        MovingPIDController.setP(DriveConstants.MovingP);
        MovingPIDController.setI(DriveConstants.MovingI);
        MovingPIDController.setD(DriveConstants.MovingD);
        MovingPIDController.setFF(DriveConstants.MovingFF);
        MovingPIDController.setOutputRange(-1.0,1.0);

        MovingSpark.burnFlash();


        TurningSpark = new CANSparkMax(TurningID, MotorType.kBrushless);
        TurningSpark.restoreFactoryDefaults();
        TurningSpark.setIdleMode(IdleMode.kBrake);
        TurningSpark.setSmartCurrentLimit(20);

        TurningEncoder = TurningSpark.getAbsoluteEncoder(Type.kDutyCycle);
        TurningEncoder.setPositionConversionFactor(2 * Math.PI);
        TurningEncoder.setInverted(true);

        TurningPIDController = TurningSpark.getPIDController();
        TurningPIDController.setFeedbackDevice(TurningEncoder);
        /*These next 3 lines tell the PID controller for the turning motor that the position 0 and 
        2 pi are the same, which allows it to move to any position in radians as quickly as possible.*/
        TurningPIDController.setPositionPIDWrappingEnabled(true);
        TurningPIDController.setPositionPIDWrappingMinInput(0);
        TurningPIDController.setPositionPIDWrappingMinInput(2 * Math.PI);

        //PID values for the turning wheel
        TurningPIDController.setP(DriveConstants.TurningP);
        TurningPIDController.setI(DriveConstants.TurningI);
        TurningPIDController.setD(DriveConstants.TurningD);
        TurningPIDController.setOutputRange(-1.0,1.0);

        TurningSpark.burnFlash();

        AngleOffset = Offset;

        //create and start the timer for the module, we only really use the timer for the module velocity.

    }



    public double TurningPosition(){
        return((TurningEncoder.getPosition() + AngleOffset) * -1);
        /* This returns the position turning wheel depending on what position is is 
        facing in radians. We take account for the offset of the angle, and then multiply by -1
        because in order for the PID controller to work, we had to turn inverted on, so this just 
        uninverts it.*/
    }

    public double DrivingPosition(){
        return(MovingEncoder.getPosition());
        //returns the posistion of the motor in feet.
    }


    private double WantedPosition;
    private double CurrentPosition;
    private double IsReversed;

    public void Run(double ModuleMagnitude, double ModuleDirection){
        /*This will see where the direction of the wheel will be.
        If it's faster, the wheel will point in the opposite direction and spin
        the wheel the opposite way.*/
        WantedPosition = Math.IEEEremainder(ModuleDirection, 2 * Math.PI);
        CurrentPosition = Math.IEEEremainder(TurningPosition(), 2 * Math.PI);
        if((Math.abs(CurrentPosition - WantedPosition) < Math.toRadians(90)) || (Math.abs(CurrentPosition - WantedPosition) > Math.toRadians(270))){
            /*The biggest possible turn the wheel should take is 90 degrees, so we see if the gap
            is less than 90 degrees or greater then 270 degrees(90 in the opposite direction). if it is 
            then we set all of our values to be normal. */
            IsReversed = 1;
            SetPosition = WantedPosition;
        }else{
            /*If it isn't, then we set the varialbe IsReversed to negative one, which will multiply by 
            the velocity we want to be the opposite, and then we set the variable SetPosition to the 
            opposite of the direction we originally wanted by adding 180 degrees to it.*/
            IsReversed = -1;
            SetPosition = WantedPosition + Math.toRadians(180);
        }

            //We set the wheel ot the direction (Turning) and magnitude (Moving) we want.
            MovingPIDController.setReference(ModuleMagnitude * IsReversed, CANSparkMax.ControlType.kVelocity);
            TurningPIDController.setReference((SetPosition + AngleOffset) * -1, CANSparkMax.ControlType.kPosition);
            //the reason why we multiply by -1 is explained in the TurningPosition() function up above.
        }
    


    private double[] ModuleCoords = {0, 0};
    private double PastPosition = 0;

    public double[] ModulePosition(double Gyro){
        /*This function returns the position of the robot in a list.
        The values in order is: X Position, Y Position, and the Velocity of the robot.*/
        double CurrentPosition = DrivingPosition();
        double PositionChange = CurrentPosition - PastPosition;
        double ModuleDirection = TurningPosition() + Math.toRadians(Gyro);
        ModuleCoords[0] += Math.sin(ModuleDirection) * PositionChange;
        ModuleCoords[1] += Math.cos(ModuleDirection) * PositionChange;

        PastPosition = CurrentPosition;
        return(ModuleCoords);
    }

    public void ResetModulePosition(){
        ModuleCoords[0] = 0;
        ModuleCoords[1] = 0;
    }

}