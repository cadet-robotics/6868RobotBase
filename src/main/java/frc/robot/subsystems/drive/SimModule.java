package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimModule implements SwerveModule {
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private final SelfControlledModuleSimulation simModule;
    private final SwerveModuleSimulation simModuleManaged;
    
    public SimModule(int driveMotorID, int angleMotorID, Rotation2d chassisAngularOffset) {

        SwerveModuleSimulationConfig simModuleConfig = SwerveDriveConstants.getSimConfig(driveMotorID, angleMotorID);
        simModuleManaged = new SwerveModuleSimulation(simModuleConfig);
        simModule = new SelfControlledModuleSimulation(simModuleManaged);

        simModule.withSteerPID(SwerveDriveConstants.simAnglePID);
        simModule.withCurrentLimits(Amps.of(40), Amps.of(20));
    }

    @Override
    public SwerveModuleState getState() {
        return simModule.getMeasuredState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return simModule.getModulePosition();
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        simModule.optimizeAndRunModuleState(desiredState);
        this.desiredState = desiredState;
    }

    @Override 
    public SwerveModuleSimulation getSimModule() {
        return simModuleManaged;
    }

}
