package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
public class LiftSubsystem extends SubsystemBase{

    public CANSparkMax LeftSlide;
    public RelativeEncoder LeftSlideEncoder;
    public SparkMaxPIDController LeftSlidePIDController;

    public CANSparkMax RightSlide;
    public RelativeEncoder RightSlideEncoder;
    public SparkMaxPIDController RightSlidePIDController;

    public LiftSubsystem(){
        LeftSlide = new CANSparkMax(14, MotorType.kBrushless);

        LeftSlide.restoreFactoryDefaults();
        LeftSlide.setSmartCurrentLimit(40);
        LeftSlideEncoder = LeftSlide.getEncoder();
        LeftSlidePIDController = LeftSlide.getPIDController();
        LeftSlidePIDController.setFeedbackDevice(LeftSlideEncoder);
        LeftSlideEncoder.setPositionConversionFactor(16 * 3 * Math.PI);
        LeftSlideEncoder.setPosition(0);
        LeftSlide.setIdleMode(IdleMode.kBrake);

        LeftSlidePIDController.setP(1);
        LeftSlidePIDController.setI(0);
        LeftSlidePIDController.setD(0);

        RightSlide = new CANSparkMax(15, MotorType.kBrushless);

        RightSlide.restoreFactoryDefaults();
        RightSlide.setSmartCurrentLimit(40);
        RightSlideEncoder = RightSlide.getEncoder();
        RightSlidePIDController = RightSlide.getPIDController();
        RightSlidePIDController.setFeedbackDevice(RightSlideEncoder);
        RightSlideEncoder.setPositionConversionFactor(16 * 3 * Math.PI);
        RightSlideEncoder.setPosition(0);
        RightSlide.setIdleMode(IdleMode.kBrake);

        RightSlidePIDController.setP(1);
        RightSlidePIDController.setI(0);
        RightSlidePIDController.setD(0);
    }

    public void SlideFoward(){
        LeftSlide.set(0.1);
        RightSlide.set(-0.1);

        //RightSlidePIDController.setReference(5, CANSparkMax.ControlType.kPosition);
        //LeftSlidePIDController.setReference(-5, CANSparkMax.ControlType.kPosition);
    }
    public void SlideBackward(){
        LeftSlide.set(-0.1);
        RightSlide.set(0.1);
    }

    public void StopSlide(){
        LeftSlide.set(0);
        RightSlide.set(0);
    }

}
