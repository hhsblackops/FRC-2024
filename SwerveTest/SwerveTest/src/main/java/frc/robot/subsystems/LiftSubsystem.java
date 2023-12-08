package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import com.revrobotics.CANSparkMax.IdleMode;

public class LiftSubsystem extends SubsystemBase{

    private CANSparkMax RightSlideSpark, LeftSlideSpark; //15, 14
    private RelativeEncoder RightSlideEncoder, LeftSlideEncoder;
    private SparkMaxPIDController RightSlidePIDController, LeftSlidePIDController;

    private CANSparkMax RightArmSpark; //7, 6
    private RelativeEncoder RightArmEncoder;
    private SparkMaxPIDController RightArmPIDController;

    private CANSparkMax RightGrabberSpark, LeftGrabberSpark; //5, 4



    public LiftSubsystem(){

        //SLIDES
        RightSlideSpark = new CANSparkMax(15, MotorType.kBrushless);

        RightSlideSpark.restoreFactoryDefaults();
        RightSlideSpark.setSmartCurrentLimit(40);
        RightSlideEncoder = RightSlideSpark.getEncoder();
        RightSlidePIDController = RightSlideSpark.getPIDController();
        RightSlidePIDController.setFeedbackDevice(RightSlideEncoder);
        RightSlidePIDController.setOutputRange(-0.3, 0.3);
        RightSlideEncoder.setPositionConversionFactor((3 * Math.PI) / 16);
        RightSlideEncoder.setPosition(0);
        RightSlideSpark.setIdleMode(IdleMode.kBrake);
        RightSlideSpark.burnFlash();

        LeftSlideSpark = new CANSparkMax(14, MotorType.kBrushless);

        LeftSlideSpark.restoreFactoryDefaults();
        LeftSlideSpark.setSmartCurrentLimit(40);
        LeftSlideEncoder = LeftSlideSpark.getEncoder();
        LeftSlidePIDController = LeftSlideSpark.getPIDController();
        LeftSlidePIDController.setFeedbackDevice(LeftSlideEncoder);
        LeftSlidePIDController.setOutputRange(-0.4, 0.4);
        LeftSlideEncoder.setPositionConversionFactor((3 * Math.PI) / 16);
        LeftSlideEncoder.setPosition(0);
        LeftSlideSpark.setIdleMode(IdleMode.kBrake);
        LeftSlideSpark.burnFlash();


        //ARM
        RightArmSpark = new CANSparkMax(7, MotorType.kBrushless);
        
        RightArmSpark.restoreFactoryDefaults();
        RightArmSpark.setSmartCurrentLimit(40);
        RightArmEncoder = RightArmSpark.getEncoder();
        RightArmPIDController = RightArmSpark.getPIDController();
        RightArmPIDController.setFeedbackDevice(RightArmEncoder);
        RightArmPIDController.setOutputRange(-0.3, 0.3);
        RightArmEncoder.setPositionConversionFactor(360 / 125);
        RightArmEncoder.setPosition(0);
        RightArmSpark.setIdleMode(IdleMode.kBrake);
        RightArmSpark.burnFlash();



        //GRABBER
        RightGrabberSpark = new CANSparkMax(5, MotorType.kBrushless);
        RightGrabberSpark.setSmartCurrentLimit(20);

        LeftGrabberSpark = new CANSparkMax(4, MotorType.kBrushless);
        LeftGrabberSpark.setSmartCurrentLimit(20);

        
    }


    public void SetCoast(){
        RightArmSpark.set(0);
        RightArmSpark.setIdleMode(IdleMode.kCoast);
    }

    public void SetSlidePID(double kP, double kI, double kD){
        LeftSlidePIDController.setP(kP);
        LeftSlidePIDController.setI(kI);
        LeftSlidePIDController.setD(kD);

        RightSlidePIDController.setP(kP);
        RightSlidePIDController.setI(kI);
        RightSlidePIDController.setD(kD);
        
    }

    double RightSlidePastPosition = 0;
    double LeftSlidePastPosition = 0;

    public void SetSlideInitial(){
        RightSlidePastPosition = RightSlideEncoder.getPosition();
        LeftSlidePastPosition = LeftSlideEncoder.getPosition();
    }

    public double GetSlideAppliedOutput(){
        return((-RightSlideSpark.getAppliedOutput() + LeftSlideSpark.getAppliedOutput()) / 2);
    }
    
    public void SetSlidePosition(double Position){
        if(Position == 0){
            RightSlidePIDController.setReference((16 * SlideSide) + RightSlidePastPosition, CANSparkMax.ControlType.kPosition);
            LeftSlidePIDController.setReference((16 * SlideSide) - LeftSlidePastPosition , CANSparkMax.ControlType.kPosition);
        }else{
            RightSlideSpark.set(-0.3 * Position);
            LeftSlideSpark.set(0.3 * Position);
        }
        //RightSlidePIDController.setReference(-Position, CANSparkMax.ControlType.kPosition);
        //LeftSlidePIDController.setReference(Position, CANSparkMax.ControlType.kPosition);
    }
    double SlideSide = 0;
    public void SaveSlideSide(double SavePosition){
        SlideSide = SavePosition;

    }

    public double GetSlideSide(){
        return(SlideSide);
    }

    public double GetSlidePosition(){
        return((-RightSlideEncoder.getPosition() + LeftSlideEncoder.getPosition()) / 2);
    }

    public void StopSlide(){
        RightSlideSpark.set(0);
        LeftSlideSpark.set(0);
    }


    public void SetArmPID(double kP, double kI, double kD){
        RightArmPIDController.setP(kP);
        RightArmPIDController.setI(kI);
        RightArmPIDController.setD(kD);

    }


    public void SetArmPosition(double Position){
        RightArmSpark.setIdleMode(IdleMode.kBrake);
        RightArmPIDController.setReference(Position, CANSparkMax.ControlType.kPosition);
    }

    public void StopArm(){
        RightArmSpark.set(0);
    }



    public double GetGrabberOutputLimit(){
        return((RightGrabberSpark.getOutputCurrent() + LeftGrabberSpark.getOutputCurrent()) / 2);
    }

    public double GetSlideOutputLimit(){
        return((RightSlideSpark.getOutputCurrent() + LeftSlideSpark.getOutputCurrent()) / 2);
    }


    public void SetGrabber(double Speed){
        LeftGrabberSpark.set(Speed);
        RightGrabberSpark.set(-Speed);
    }

    public void SetSlideSpeed(double speed){
        LeftSlideSpark.set(speed);
        RightSlideSpark.set(-speed);

    }
}
