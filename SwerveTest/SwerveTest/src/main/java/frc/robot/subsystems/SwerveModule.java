// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule {
    private CANSparkMax MovingSpark;

    private CANSparkMax TurningSpark;
    private SparkMaxPIDController TurningPIDController;
    private AbsoluteEncoder TurningEncoder;
    private double SetPosition;
    private double AngleOffset;
    private double WantedPosition;
    private double CurrentPosition;
    private double IsReversed;
    public SwerveModule(int TurningID, int MovingID, double Offset){
        MovingSpark = new CANSparkMax(MovingID, MotorType.kBrushless);
        MovingSpark.restoreFactoryDefaults();
        MovingSpark.setSmartCurrentLimit(40);
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
        

    }
    public void Run(double speed, double angle){
        /* This next series of if statements is to reverse the motor if 
         * it is closer to switch direction and reverse the motor.
         */

        WantedPosition = Math.IEEEremainder(angle, 2 * Math.PI);
        CurrentPosition = Math.IEEEremainder((TurningEncoder.getPosition() * -1) - AngleOffset, 2 * Math.PI);
        if((Math.abs(CurrentPosition - WantedPosition) < Math.PI / 2) ||
        (Math.abs(CurrentPosition - WantedPosition) > 3 * Math.PI / 2)){
            IsReversed = 1;
            SetPosition = WantedPosition;
        }else{
            IsReversed = -1;
            SetPosition = WantedPosition + Math.PI;
        }



        TurningPIDController.setReference((SetPosition + AngleOffset) * -1, CANSparkMax.ControlType.kPosition);
        MovingSpark.set(speed * IsReversed);
        
    }
}

