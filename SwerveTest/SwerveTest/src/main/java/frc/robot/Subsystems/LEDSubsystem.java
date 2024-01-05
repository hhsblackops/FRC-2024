package frc.robot.Subsystems;


import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class LEDSubsystem extends SubsystemBase{

    private Spark Blinkin = new Spark(LEDConstants.LEDPort);

    public void SetColor(double Color){
        Blinkin.set(Color);
    }

}
