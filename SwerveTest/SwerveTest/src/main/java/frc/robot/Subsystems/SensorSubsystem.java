package frc.robot.Subsystems;

import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class SensorSubsystem extends SubsystemBase{

    Spark Blinkin = new Spark(LEDConstants.LEDPort);
    I2C.Port I2CPort = I2C.Port.kOnboard;
    ColorSensorV3 ColorSensor = new ColorSensorV3(I2CPort);

    private final NetworkTable LimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    private final NetworkTableEntry HorizontalOffsetEntry = LimelightTable.getEntry("tx");
    private final NetworkTableEntry VerticalOffsetEntry = LimelightTable.getEntry("ty");
    private final NetworkTableEntry AreaEntry = LimelightTable.getEntry("ta");


    double[] LimelightValues = {0, 0, 0};

    Color DetectedColor;



    @Override
    public void periodic(){
        DetectedColor = ColorSensor.getColor();
        SmartDashboard.putNumber("tx", GetHorizontalOffset());
        SmartDashboard.putNumber("ty", GetVerticalOffset());
        SmartDashboard.putNumber("ta", GetArea());
        SmartDashboard.putBoolean("IsVisible", IsTargetVisible());

    }

    public void SetColor(double LEDColor){
        Blinkin.set(LEDColor);
    }

    public boolean IsTargetVisible(){
        return(!(AreaEntry.getDouble(0) == 0));
    }

    public double GetHorizontalOffset(){
        return(HorizontalOffsetEntry.getDouble(0));
    }

    public double GetVerticalOffset(){
        return(VerticalOffsetEntry.getDouble(0));
    }

    public double GetArea(){
        return(AreaEntry.getDouble(0));
    }

    public Color GetColor(){
        return(DetectedColor);
    }

}