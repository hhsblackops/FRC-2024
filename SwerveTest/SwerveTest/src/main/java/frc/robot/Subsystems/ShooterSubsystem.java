package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ShooterSubsystem extends SubsystemBase{

    CANSparkMax ShooterLeft;

    CANSparkMax ShooterRightFront;
    CANSparkMax ShooterRightBack;

    CANSparkMax ShooterIntake;

    CANSparkMax RightArm;
    RelativeEncoder RightArmEncoder;
    SparkMaxPIDController RightArmPIDController;

    CANSparkMax LeftArm;
    RelativeEncoder LeftArmEncoder;
    SparkMaxPIDController LeftArmPIDController;

    CANSparkMax IntakeTop;
    CANSparkMax IntakeBottom;

    double ShooterPower = 1;

    double ArmP = 17;
    double ArmI = 0;
    double ArmD = 0;
    double ArmMinOutPut = -0.2;
    double ArmMaxOutPut = 0.2;

    boolean HasNote;    

    public ShooterSubsystem(){
        ShooterLeft = new CANSparkMax(3, MotorType.kBrushless);
        ShooterLeft.restoreFactoryDefaults();
        ShooterLeft.setIdleMode(IdleMode.kCoast);
        ShooterLeft.setSmartCurrentLimit(40);
        ShooterLeft.burnFlash();

        ShooterRightFront = new CANSparkMax(1, MotorType.kBrushless);
        ShooterRightFront.restoreFactoryDefaults();
        ShooterRightFront.setIdleMode(IdleMode.kCoast);
        ShooterRightFront.setSmartCurrentLimit(40);
        ShooterRightFront.burnFlash();
        
        ShooterRightBack = new CANSparkMax(2, MotorType.kBrushless);
        ShooterRightBack.restoreFactoryDefaults();
        ShooterRightBack.setIdleMode(IdleMode.kCoast);
        ShooterRightBack.setSmartCurrentLimit(40);
        ShooterRightBack.burnFlash();

        ShooterIntake = new CANSparkMax(11, MotorType.kBrushless);
        ShooterIntake.restoreFactoryDefaults();
        ShooterIntake.setIdleMode(IdleMode.kBrake);
        ShooterIntake.setSmartCurrentLimit(20);
        ShooterIntake.burnFlash();

        RightArm = new CANSparkMax(14, MotorType.kBrushless);
        RightArm.restoreFactoryDefaults();
        RightArm.setIdleMode(IdleMode.kBrake);
        RightArm.setSmartCurrentLimit(40);
        RightArmEncoder = RightArm.getEncoder();
        RightArmEncoder.setPositionConversionFactor((2 * Math.PI) / 125);
        RightArmEncoder.setPosition(0);
        RightArmPIDController = RightArm.getPIDController();
        RightArmPIDController.setFeedbackDevice(RightArmEncoder);
        RightArmPIDController.setP(ArmP);
        RightArmPIDController.setI(ArmI);
        RightArmPIDController.setD(ArmD);
        RightArmPIDController.setOutputRange(ArmMinOutPut, ArmMaxOutPut);
        RightArm.burnFlash();
        RightArmPIDController.setReference(0, CANSparkMax.ControlType.kPosition);

        LeftArm = new CANSparkMax(13, MotorType.kBrushless);
        LeftArm.restoreFactoryDefaults();
        LeftArm.setIdleMode(IdleMode.kBrake);
        LeftArm.setSmartCurrentLimit(40);
        LeftArm.follow(RightArm);
        LeftArm.burnFlash();

        IntakeTop = new CANSparkMax(15, MotorType.kBrushless);
        IntakeTop.restoreFactoryDefaults();
        IntakeTop.setIdleMode(IdleMode.kBrake);
        IntakeTop.setSmartCurrentLimit(40);
        IntakeTop.burnFlash();

        IntakeBottom = new CANSparkMax(12, MotorType.kBrushless);
        IntakeBottom.restoreFactoryDefaults();
        IntakeBottom.setIdleMode(IdleMode.kBrake);
        IntakeBottom.setSmartCurrentLimit(40);
        IntakeBottom.burnFlash();

        HasNote = true;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Has Note", IsNote());
        SmartDashboard.putNumber("Arm Angle", GetArmPosition());
    }

    public void NoteGone(){
        HasNote = false;
    }
    
    public void NoteHere(){
        HasNote = true;
    }

    public boolean IsNote(){
        return(HasNote);
    }

    public void Shoot(){
        ShooterLeft.set(-ShooterPower);
        ShooterRightFront.set(ShooterPower);
        ShooterRightBack.set(ShooterPower);
    }

    public void ShooterOff(){
        ShooterLeft.disable();

        ShooterRightFront.disable();
        ShooterRightBack.disable();
    }

    public void SetArmPosition(double Position){
        RightArmPIDController.setReference(-Position, CANSparkMax.ControlType.kPosition);
    }

    public double GetArmPosition(){
        return(-(RightArmEncoder.getPosition()));
    }

    public void MoveArmUp(double direction){

        RightArmPIDController.setReference(RightArmEncoder.getPosition() - Math.toRadians(5) * direction, CANSparkMax.ControlType.kPosition);
    }

    public void StopArm(){
        RightArmPIDController.setReference(RightArmEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
    }

    public void SetShooterIntake(double Speed){
        ShooterIntake.set(Speed);
    }

    public void SetIntake(double Speed){
        IntakeBottom.set(Speed * 0.25);
        IntakeTop.set(Speed);
    }
    
}