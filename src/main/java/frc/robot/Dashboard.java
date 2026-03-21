package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;


public class Dashboard {
    public enum Alliance {
        Red, 
        Blue, 
        NotConnected,
        PoorlyWrittenCode
    }    

    public Dashboard.Alliance getAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return Alliance.NotConnected;
        }
        switch (alliance.get()) {
            case Red:
                return Alliance.Red;
            case Blue:
                return Alliance.Blue;
        }
        return Alliance.PoorlyWrittenCode;
    } 
}
