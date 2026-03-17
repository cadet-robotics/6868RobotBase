// Swerve Drive with a Quadrants 2,1,3,4 layout.
package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.command.BetterSubsystem;

public class SwerveDrive extends BetterSubsystem {
    public static enum LogLevel {
        None,
        Pose,
        Verbose
    }

    private LinearVelocity maxLinearVelocity = SwerveDriveConstants.maxLinearVelocity;
    private final AHRS gyro = new AHRS(NavXComType.kMXP_UART);

    private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        SwerveDriveConstants.frontLeftLocation,
        SwerveDriveConstants.frontRightLocation,
        SwerveDriveConstants.backLeftLocation,
        SwerveDriveConstants.backRightLocation
    );

    private SwerveDriveOdometry swerveDriveOdometry = new SwerveDriveOdometry(
        swerveKinematics, 
        getRotation(), 
        new SwerveModulePosition[] {
            SwerveDriveConstants.frontLeftPosition,
            SwerveDriveConstants.frontRightPosition,
            SwerveDriveConstants.backLeftPosition,
            SwerveDriveConstants.backRightPosition
        }
    );

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics, 
        getRotation(), 
        new SwerveModulePosition[] {
            SwerveDriveConstants.frontLeftPosition,
            SwerveDriveConstants.frontRightPosition,
            SwerveDriveConstants.backLeftPosition,
            SwerveDriveConstants.backRightPosition,
        },
        getPose()
    );

    public SwerveDrive() {
        configureAutobuilder();
    }

    public void configureAutobuilder() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // If config file loading fails, report and return without configuring
            DriverStation.reportError(
                    "Failed to load PathPlanner config: " + e.getMessage(),
                    e.getStackTrace()
            );
            return;
        }

        AutoBuilder.configure(
            this::getPose,
            this::setOdometry,
            this::getChassisSpeeds,
            (ChassisSpeeds robotRelativeSpeeds, DriveFeedForwards moduleFeedForwards) -> {
                drive(robotRelativeSpeeds, robotRelativeSpeeds. );
            }
            , null, null, null, null);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return 
    }

    public LinearVelocity getVelocity() {

    }

    public void setMaxVelocity(Velocity velo)

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);

    public void drive(double xSpeed, double ySpeed, Rotation2d heading, boolean fieldRelative);

    /**
     * Drive method for PathPlanner or teleop control.
     * @param chassisSpeeds Desired robot chassis speeds (vx, vy, omega)
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        // Convert chassis speeds to individual module states
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize wheel speeds to prevent saturation
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearVelocity.magnitude());

        // Apply states to each module
        frontLeft.set(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRight.set(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeft.set(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRight.set(states[3].speedMetersPerSecond, states[3].angle.getRadians());

        
    }
}
    public ChassisSpeeds getChassisSpeeds();
    public void setModuleStates(SwerveModuleState[] desiredStates);
    public void setOdometry(Pose2d pose);
    public PoseEstimator<Pose2d> getPoseEstimator();
    public void setLogLevel(SwerveDrive.LogLevel logLevel);

}