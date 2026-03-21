package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro<T> {
    public Rotation2d getRotation();
    static <T> T getInstance() {
        throw new UnsupportedOperationException();
    };
}