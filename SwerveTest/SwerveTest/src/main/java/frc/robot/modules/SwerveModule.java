package frc.robot.modules;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Timer;


public class SwerveModule {
    
    private CANSparkMax MovingSpark;
    private SparkMaxPIDController MovingPIDController;

    private CANSparkMax TurningSpark;
    private SparkMaxPIDController TurningPIDController;
    private AbsoluteEncoder TurningEncoder;
    private RelativeEncoder MovingEncoder;
    private double SetPosition;
    private double AngleOffset;

    private Timer Timer;
    

    public SwerveModule(int TurningID, int MovingID, double Offset){
        MovingSpark = new CANSparkMax(MovingID, MotorType.kBrushless);
        MovingSpark.restoreFactoryDefaults();
        MovingSpark.setSmartCurrentLimit(40);
        MovingEncoder = MovingSpark.getEncoder();
        MovingPIDController = MovingSpark.getPIDController();
        MovingPIDController.setFeedbackDevice(MovingEncoder);
        MovingEncoder.setPositionConversionFactor(DriveConstants.DrivingPositionFactor);
        MovingEncoder.setVelocityConversionFactor(DriveConstants.DrivingVelocityFactor);
        MovingEncoder.setPosition(0);


        //MovingPIDController.setP(0);
        MovingPIDController.setI(0);
        MovingPIDController.setD(0);
        //MovingPIDController.setFF(0);
        MovingPIDController.setOutputRange(-1.0,1.0);

        MovingSpark.burnFlash();


        TurningSpark = new CANSparkMax(TurningID, MotorType.kBrushless);
        TurningSpark.restoreFactoryDefaults();
        TurningEncoder = TurningSpark.getAbsoluteEncoder(Type.kDutyCycle);
        TurningPIDController = TurningSpark.getPIDController();
        TurningPIDController.setFeedbackDevice(TurningEncoder);
        TurningEncoder.setPositionConversionFactor(2 * Math.PI);
        TurningEncoder.setInverted(true);
        TurningPIDController.setPositionPIDWrappingEnabled(true);
        TurningPIDController.setPositionPIDWrappingMinInput(0);
        TurningPIDController.setPositionPIDWrappingMinInput(2 * Math.PI);


        TurningPIDController.setP(1);
        TurningPIDController.setI(0);
        TurningPIDController.setD(0);
        TurningPIDController.setOutputRange(-1.0,1.0);


        TurningSpark.setIdleMode(IdleMode.kBrake);
        TurningSpark.setSmartCurrentLimit(20);


        TurningSpark.burnFlash();
        AngleOffset = Offset;


        Timer = new Timer();
        Timer.start();
    }

    public void PIDTuning(double kP, double kF){
        MovingPIDController.setP(kP);
        MovingPIDController.setFF(kF);
    }

    public double TurningPosition(){
        return((TurningEncoder.getPosition() * -1) - AngleOffset);
    }
    public double DrivingPosition(){
        return(MovingEncoder.getPosition());
    }


    private double WantedPosition;
    private double CurrentPosition;
    private double IsReversed;

    public void Run(double ModuleMagnitude, double ModuleDirection){
        /* This next series of if statements is to reverse the wheel direction
         * it is closer to drive the wheel backwards in the opposite direction
         */

        WantedPosition = Math.IEEEremainder(ModuleDirection, 2 * Math.PI);
        CurrentPosition = Math.IEEEremainder(TurningPosition(), 2 * Math.PI);
        if((Math.abs(CurrentPosition - WantedPosition) < Math.PI / 2) ||
        (Math.abs(CurrentPosition - WantedPosition) > 3 * Math.PI / 2)){
            IsReversed = 1;
            SetPosition = WantedPosition;
        }else{
            IsReversed = -1;
            SetPosition = WantedPosition + Math.PI;
        }

        TurningPIDController.setReference((SetPosition + AngleOffset) * -1, CANSparkMax.ControlType.kPosition);
        MovingPIDController.setReference(ModuleMagnitude * IsReversed, CANSparkMax.ControlType.kVelocity);
    }


    private double[] ModuleCoords = {0, 0, 0};
    private double PastPosition = 0;
    private double PastTime = 0;

    public double[] ModulePosition(double Gyro){
        double CurrentPosition = DrivingPosition();
        double PositionChange = CurrentPosition - PastPosition;
        double ModuleDirection = TurningPosition() - Math.toRadians(Gyro);
        ModuleCoords[0] += Math.sin(ModuleDirection) * PositionChange; //First returned value is X Position
        ModuleCoords[1] += Math.cos(ModuleDirection) * PositionChange; //Second returned value is Y Position

        double CurrentTime = Timer.get();
        double TimeChange = CurrentTime - PastTime;
        double WheelVelocity = PositionChange / TimeChange;
        ModuleCoords[2] = WheelVelocity;

        PastPosition = CurrentPosition;
        PastTime = CurrentTime;
        return(ModuleCoords);

    }

    public void ResetModulePosition(){
        ModuleCoords[0] = 0;
        ModuleCoords[1] = 0;
    }

    



}