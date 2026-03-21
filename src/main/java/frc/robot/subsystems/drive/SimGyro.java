package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimGyro implements Gyro {

    private GyroSimulation gyro;

    public SimGyro() {
        gyro = new GyroSimulation(0.025, 65);
    }

    
    public GyroSimulation getSimGyro() {
        return gyro;
    }

    @Override
    public Rotation2d getRotation() {
        return gyro.getGyroReading();
    }

}
