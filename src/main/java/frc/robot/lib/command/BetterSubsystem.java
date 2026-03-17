package frc.robot.lib.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BetterSubsystem extends SubsystemBase {
    public BetterSubsystem() {
        super();
    }

    /**
     * @return {@link CommandBuilder} - A new CommandBuilder with this subsystem as a requirement.
     */
    public CommandBuilder commandBuilder() {
        return new CommandBuilder(this);
    }

    /**
     * @param name The name of the command.
     * @return {@link CommandBuilder} - A new CommandBuilder with this subsystem as a requirement.
     */
    public CommandBuilder commandBuilder(String name) {
        return new CommandBuilder(name, this);
    }

    /**
     * Called periodically during autonomous. Override this method to run code during the autonomous period.
     * If enabledPeriodic is overridden, this method must be called manually in order for this method to run.
     */
    public void autonomousPeriodic() {}

    /**
     * Called periodically during teleop. Override this method to run code during the teleop period.
     * If enabledPeriodic is overridden, this method must be called manually in order for this method to run.
     */
    public void teleopPeriodic() {}

    /**
     * Called periodically while the robot is disabled. Override this method to run code while the robot is disabled.
     * If periodic is overridden, this method must be called manually in order for this method to run.
     */
    public void disabledPeriodic() {}

    /**
     * Called periodically while the robot is enabled. 
     * Calls: Teleop periodic
     */
    public void enabledPeriodic() {
        if (DriverStation.isAutonomous()) {
            autonomousPeriodic();
        } else if (DriverStation.isTeleop()) {
            teleopPeriodic();
        }
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            disabledPeriodic();
        } else if (DriverStation.isEnabled()) {
            enabledPeriodic();
        }
    }

}