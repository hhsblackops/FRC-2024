package frc.robot.Modules;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase{
    /*private final NetworkTable LimelightTable;
    private final NetworkTableEntry TargetVisibleEntry;
    private final NetworkTableEntry HorizontalOffsetEntry;
    private final NetworkTableEntry VerticalOffsetEntry;
    private final NetworkTableEntry AreaEntry;*/

    public Limelight(String CameraName){
        /*LimelightTable = NetworkTableInstance.getDefault().getTable(CameraName);
        TargetVisibleEntry = LimelightTable.getEntry("tv");
        HorizontalOffsetEntry = LimelightTable.getEntry("tx");
        VerticalOffsetEntry = LimelightTable.getEntry("ty");
        AreaEntry = LimelightTable.getEntry("ta");*/

    }

    public boolean IsTargetVisible(){
        return(false);//TargetVisibleEntry.getBoolean(false));
    }

    public double GetHorizontalOffset(){
        return(0);//HorizontalOffsetEntry.getDouble(0));
    }

    public double GetVerticalOffset(){
        return(0);//VerticalOffsetEntry.getDouble(0));
    }

    public double GetArea(){
        return(0);//AreaEntry.getDouble(0));
    }
}