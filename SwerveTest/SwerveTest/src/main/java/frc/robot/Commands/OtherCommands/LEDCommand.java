package frc.robot.Commands.OtherCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LEDSubsystem;

public class LEDCommand extends CommandBase{


    private final double Color;
    private final LEDSubsystem ledSubsystem;


    public LEDCommand(LEDSubsystem ledSubsystem, double Color){
        this.Color = Color;
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void initialize(){  
        ledSubsystem.SetColor(Color);
    }


  
    @Override
    public boolean isFinished(){
        return(true);
        
    }
}