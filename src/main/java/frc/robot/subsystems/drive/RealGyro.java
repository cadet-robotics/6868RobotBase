package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class RealGyro implements Gyro<RealGyro> {
    private final AHRS gyro;
    private static RealGyro instance = new RealGyro();

    public RealGyro() {
        gyro = new AHRS(NavXComType.kMXP_UART);
        instance = this;
    }

    @Override
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    public static RealGyro getInstance() {
        if (instance == null) {
            instance = new RealGyro();
        }
        return instance;
    }
}
