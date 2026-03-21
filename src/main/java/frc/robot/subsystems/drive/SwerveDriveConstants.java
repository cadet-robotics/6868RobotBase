package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.FeedForwardConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class SwerveDriveConstants {
    public static final Distance moduleLeftOffset = Meter.of(0.381);
    public static final Distance moduleRightOffset = Meter.of(-0.381);
    public static final Distance moduleFrontOffset = Meter.of(0.381);
    public static final Distance moduleBackOffset = Meter.of(-0.381);

    public static final Translation2d frontLeftLocation = new Translation2d(moduleFrontOffset.magnitude(), moduleLeftOffset.magnitude());
    public static final Translation2d frontRightLocation = new Translation2d(moduleFrontOffset.magnitude(), moduleRightOffset.magnitude());
    public static final Translation2d backLeftLocation = new Translation2d(moduleBackOffset.magnitude(), moduleLeftOffset.magnitude());
    public static final Translation2d backRightLocation = new Translation2d(moduleBackOffset.magnitude(), moduleRightOffset.magnitude());

    public static final Distance chassisWidth = moduleLeftOffset.minus(moduleRightOffset);
    public static final Distance chassisLength = moduleFrontOffset.minus(moduleBackOffset);

    public static final Distance bumperThickness = Meter.of(0.05);
    public static final Distance bumperWidth = chassisWidth.plus(bumperThickness.times(2));
    public static final Distance bumperLength = chassisLength.plus(bumperThickness.times(2));

    public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(5.0);
    public static final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(2.5);

    public static final Distance driveBaseRadius = Meter.of(Math.hypot(chassisLength.magnitude() / 2, chassisWidth.magnitude() / 2));

    public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(
        maxLinearVelocity.magnitude() / driveBaseRadius.magnitude()
    );
    public static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(
        maxLinearAcceleration.magnitude() / driveBaseRadius.magnitude()
    );

    public static final PIDController translationalPID = new PIDController(1.0, 0.0, 0.0);
    public static final PIDController rotationalPID = new PIDController(1.0, 0.0, 0.0);
    {
        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static final PIDController simTranslationalPID = new PIDController(1.0, 0.0, 0.0);
    public static final PIDController simRotationalPID = new PIDController(1.0, 0.0, 0.0);
    {
        simRotationalPID.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    public static final com.pathplanner.lib.config.PIDConstants pathplannerTranslationalPID = new com.pathplanner.lib.config.PIDConstants(
        translationalPID.getP(),
        translationalPID.getI(),
        translationalPID.getD()
    );
    public static final com.pathplanner.lib.config.PIDConstants pathplannerRotationalPID = new com.pathplanner.lib.config.PIDConstants(
        rotationalPID.getP(),
        rotationalPID.getI(),
        rotationalPID.getD()
    );

    public static final Rotation2d frontLeftAngle = frontLeftLocation.getAngle();
    public static final Rotation2d frontRightAngle = frontRightLocation.getAngle();
    public static final Rotation2d backLeftAngle = backLeftLocation.getAngle();
    public static final Rotation2d backRightAngle = backRightLocation.getAngle();

    public static final double driveGearRatio = 4.71;
    public static final double angleGearRatio = 15.43;
    public static final MomentOfInertia driveMoi = KilogramSquareMeters.of(17.493);
    public static final MomentOfInertia angleMoi = KilogramSquareMeters.of(0.1);
    public static final Voltage angleFrictionVoltage = Volts.of(0);
    public static final Voltage driveFrictionVoltage = Volts.of(0);
    public static final boolean absoluteEncoderInverted = true;
    public static final boolean driveMotorInverted = true;

    public static final AngularVelocity maxAngleMotorVelocity = RPM.of(3000);
    public static final AngularAcceleration maxAngleMotorAcceleration = RotationsPerSecondPerSecond.of(3000);
    public static final int encoderResolution = 4096;

    public static final PIDController drivePID = new PIDController(1.0, 0.0, 0.0);
    public static final ProfiledPIDController anglePID = new ProfiledPIDController(
        1.0,
        0.0,
        0.0,
        new Constraints(
            maxAngleMotorVelocity.magnitude(),
            maxAngleMotorAcceleration.magnitude()
        )
    );
    public static final PIDController simDrivePID = new PIDController(1.0, 0.0, 0.0);
    public static final PIDController simAnglePID = new PIDController( 1.0, 0.0, 0.0 );

    public static final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( 0, 0, 0 );
    public static final SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward( 0, 0, 0 );
    public static final SimpleMotorFeedforward simDriveFeedforward = new SimpleMotorFeedforward( 0, 0, 0);
    public static final SimpleMotorFeedforward simAngleFeedforward = new SimpleMotorFeedforward( 0, 0, 0);

    public static final int frontLeftDriveCanID = 1;
    public static final int frontRightDriveCanID = 2;
    public static final int backLeftDriveCanID = 3;
    public static final int backRightDriveCanID = 4;
    public static final int frontLeftAngleCanID = 5;
    public static final int frontRightAngleCanID = 6;
    public static final int backLeftAngleCanID = 7;
    public static final int backRightAngleCanID = 8;

    public static Mass robotMass;
    public static Distance wheelRadius = Inches.of(1.5);
    public static MomentOfInertia steerRotationalInertia;

    public static double wheelCoefficientOfFriction;

    public static SwerveModuleSimulationConfig getSimConfig(int driveCanID, int angleCanID) {
        return new SwerveModuleSimulationConfig(
            DCMotor.getNeoVortex(driveCanID),
            DCMotor.getNeo550(angleCanID),
            SwerveDriveConstants.driveGearRatio,
            SwerveDriveConstants.angleGearRatio,
            SwerveDriveConstants.driveFrictionVoltage,
            SwerveDriveConstants.angleFrictionVoltage,
            SwerveDriveConstants.wheelRadius,
            SwerveDriveConstants.steerRotationalInertia,
            SwerveDriveConstants.wheelCoefficientOfFriction
        );

    }
}
