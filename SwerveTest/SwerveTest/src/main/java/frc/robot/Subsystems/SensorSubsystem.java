package frc.robot.Subsystems;

import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.Modules.Limelight;



public class SensorSubsystem extends SubsystemBase{

    Spark Blinkin = new Spark(LEDConstants.LEDPort);
    public final Limelight LimelightCamera = new Limelight("limelight");

    double[] LimelightValues = {0, 0, 0};

    public void SetColor(double Color){
        Blinkin.set(Color);
    }

    public boolean GetIsTargetVisible(){
        return(LimelightCamera.IsTargetVisible());
    }

    public double[] GetLimelightValues(){
        /*This function will give you the values of the camera in a list. The values are first the 
        horizontal offset of the targer, then vertical, and then the area of the target.*/
        LimelightValues[0] = LimelightCamera.GetHorizontalOffset();
        LimelightValues[1] = LimelightCamera.GetVerticalOffset();
        LimelightValues[2] = LimelightCamera.GetArea();
        return(LimelightValues);
    }
}