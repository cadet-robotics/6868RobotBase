package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.command.BetterSubsystem;

public class SwerveDrive extends BetterSubsystem {
    public static enum LogLevel {
        None,
        Pose,
        Verbose
    }

    private double maxLinearVelocity = SwerveDriveConstants.maxLinearVelocity.in(MetersPerSecond);
    private final Gyro gyro;

    private SwerveDriveKinematics swerveKinematics;
    private SwerveDriveOdometry swerveDriveOdometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public SwerveDrive() {
        configureAutobuilder();

        if (Robot.isReal()) {
            gyro = new RealGyro();
            frontLeftModule = new RealModule(
                SwerveDriveConstants.frontLeftDriveCanID,
                SwerveDriveConstants.frontLeftAngleCanID,
                getRotation()
            );
            frontRightModule = new RealModule(
                SwerveDriveConstants.frontRightDriveCanID,
                SwerveDriveConstants.frontRightAngleCanID,
                getRotation()
            );
            backLeftModule = new RealModule(
                SwerveDriveConstants.backLeftDriveCanID,
                SwerveDriveConstants.backLeftAngleCanID,
                getRotation()
            );
            backRightModule = new RealModule(
                SwerveDriveConstants.backRightDriveCanID,
                SwerveDriveConstants.backRightAngleCanID,
                getRotation()
            );
        } else {
            frontLeftModule = new SimModule(
                SwerveDriveConstants.frontLeftDriveCanID,
                SwerveDriveConstants.frontLeftAngleCanID,
                getRotation()
            );
            frontRightModule = new SimModule(
                SwerveDriveConstants.frontRightDriveCanID,
                SwerveDriveConstants.frontRightAngleCanID,
                getRotation()
            );
            backLeftModule = new SimModule(
                SwerveDriveConstants.backLeftDriveCanID,
                SwerveDriveConstants.backLeftAngleCanID,
                getRotation()
            );
            backRightModule = new SimModule(
                SwerveDriveConstants.backRightDriveCanID,
                SwerveDriveConstants.backRightAngleCanID,
                getRotation()
            );

            DriveTrainSimulationConfig driveSimConfig = new DriveTrainSimulationConfig(
                SwerveDriveConstants.robotMass,
                SwerveDriveConstants.bumperLength,
                SwerveDriveConstants.bumperWidth,
                SwerveDriveConstants.chassisLength,
                SwerveDriveConstants.chassisWidth,
                () -> new SimGyro().getSimGyro(),
                () -> frontLeftModule.getSimModule(),
                () -> frontRightModule.getSimModule(),
                () -> backLeftModule.getSimModule(),
                () -> backRightModule.getSimModule()
            );
            SwerveDriveSimulation swerveSimManaged = new SwerveDriveSimulation(driveSimConfig, getPose());
            SelfControlledSwerveDriveSimulation swerveSim = new SelfControlledSwerveDriveSimulation(swerveSimManaged);
        }
        poseEstimator = new SwerveDrivePoseEstimator(
            swerveKinematics, 
            getRotation(), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            getPose()
        );
        swerveDriveOdometry = new SwerveDriveOdometry(
            swerveKinematics, 
            getRotation(), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );
        swerveKinematics = new SwerveDriveKinematics(
            SwerveDriveConstants.frontLeftLocation,
            SwerveDriveConstants.frontRightLocation,
            SwerveDriveConstants.backLeftLocation,
            SwerveDriveConstants.backRightLocation
        );

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

        BooleanSupplier isRedAlliance = () -> { return !(DriverStation.getAlliance().get() == Alliance.Blue); };

        AutoBuilder.configure(
            this::getPose,
            this::setOdometry,
            this::getChassisSpeeds,
            (robotRelativeSpeeds, moduleFeedForwards) -> {
                drive(robotRelativeSpeeds);
            },
            new PPHolonomicDriveController(
                SwerveDriveConstants.pathplannerTranslationalPID,
                SwerveDriveConstants.pathplannerRotationalPID
            ), 
            config,
            isRedAlliance,
            this
        );
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return gyro.getRotation();
    }

    public LinearVelocity getVelocity() {
    }

    public void setMaxVelocity(LinearVelocity velo) {
        this.maxLinearVelocity = velo.in(MetersPerSecond);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        new ChassisSpeeds(MetersPerSecond.of(xSpeed), ySpeed, DegreesPerSecond.of(rot));
    }

    public void drive(double xSpeed, double ySpeed, Rotation2d heading, boolean fieldRelative);

    /**
     * Drive method for PathPlanner or teleop control.
     * @param chassisSpeeds Desired robot chassis speeds (vx, vy, omega)
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        // Convert chassis speeds to individual module states
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize wheel speeds to prevent saturation
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearVelocity);

        // Apply states to each module
        frontLeftModule.setDesiredState(new SwerveModuleState(states[0].speedMetersPerSecond, states[0].angle));
        frontRightModule.setDesiredState(new SwerveModuleState(states[1].speedMetersPerSecond, states[1].angle));
        backLeftModule.setDesiredState(new SwerveModuleState(states[2].speedMetersPerSecond, states[2].angle));
        backRightModule.setDesiredState(new SwerveModuleState(states[3].speedMetersPerSecond, states[3].angle));

        
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

    public void setOdometry(Pose2d pose) {
        swerveDriveOdometry.resetPose(pose);
    }
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }
    public void setLogLevel(SwerveDrive.LogLevel logLevel);

}