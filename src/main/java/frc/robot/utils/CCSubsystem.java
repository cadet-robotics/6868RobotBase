package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CCommand;

public class CCSubsystem extends SubsystemBase {
    public CCSubsystem() {
        super();
    }

    public Command cCommand(String name) {
       return new CCommand(name, this);
    }
}
