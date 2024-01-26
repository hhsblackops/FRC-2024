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

    CANSparkMax ShooterSpark = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax ShooterSparkTwo = new CANSparkMax(15, MotorType.kBrushless);

    double Power;
    public ShooterSubsystem(){
        ShooterSpark.restoreFactoryDefaults();
        ShooterSpark.setIdleMode(IdleMode.kBrake);
        ShooterSpark.setSmartCurrentLimit(40);
        ShooterSpark.burnFlash();

        Power = -1;
        ShooterSparkTwo.restoreFactoryDefaults();
        ShooterSparkTwo.setIdleMode(IdleMode.kBrake);
        ShooterSparkTwo.setSmartCurrentLimit(40);
        ShooterSparkTwo.burnFlash();


    }

    public void TurnOnPos(){
        //This sucks it in as of rn
        ShooterSpark.set(Power);// * 0.1);
        ShooterSparkTwo.set(-Power * 0.9);// * 0.1);
    }

    public void TurnOnNeg(){
        ShooterSpark.set(-Power);
        ShooterSparkTwo.set(Power * 0.9);

    }

    public void TurnOff(){
        ShooterSpark.set(0);
        ShooterSparkTwo.set(0);

    }
}
