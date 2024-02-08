package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase{

    CANSparkMax ShooterLeft = new CANSparkMax(3, MotorType.kBrushless);

    CANSparkMax ShooterRightFront = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax ShooterRightBack = new CANSparkMax(2, MotorType.kBrushless);

    double Power;
    public ShooterSubsystem(){
        ShooterRightFront.restoreFactoryDefaults();
        ShooterRightFront.setIdleMode(IdleMode.kBrake);//IdleMode.kCoast);
        ShooterRightFront.setSmartCurrentLimit(40);
        ShooterRightFront.burnFlash();
        
        ShooterRightBack.restoreFactoryDefaults();
        ShooterRightBack.setIdleMode(IdleMode.kBrake);//IdleMode.kCoast);
        ShooterRightBack.setSmartCurrentLimit(40);
        ShooterRightBack.burnFlash();

        ShooterLeft.restoreFactoryDefaults();
        ShooterLeft.setIdleMode(IdleMode.kBrake);//IdleMode.kCoast);
        ShooterLeft.setSmartCurrentLimit(20);
        ShooterLeft.burnFlash();

        Power = 0.1;
    }

    public void Shoot(){
        //ShooterLeft.set(-Power);

        ShooterRightFront.set(-Power);
        ShooterRightBack.set(-Power);
    }

    public void TurnOff(){
        //ShooterLeft.disable();

        ShooterRightFront.disable();
        ShooterRightBack.disable();


    }
}
