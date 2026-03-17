package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.config.FeedForwardConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
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

    public static final ProfiledPIDController pathplannerTranslationalPID = new ProfiledPIDController(
        translationalPID.getP(),
        translationalPID.getI(),
        translationalPID.getD(),
        new Constraints(maxLinearVelocity.magnitude(), maxLinearAcceleration.magnitude())
    );
    public static final ProfiledPIDController pathplannerRotationalPID = new ProfiledPIDController(
        rotationalPID.getP(),
        rotationalPID.getI(),
        rotationalPID.getD(),
        new Constraints(maxAngularVelocity.magnitude(), maxAngularAcceleration.magnitude())
    );

    public static final Distance frontLeftDistance = Meter.of(
        Math.hypot(
            frontLeftLocation.getX(), 
            frontLeftLocation.getY()
        )
    );
    public static final Distance frontRightDistance = Meter.of(
        Math.hypot(
            frontRightLocation.getX(), 
            frontRightLocation.getY()
        )
    );
    public static final Distance backLeftDistance = Meter.of(
        Math.hypot(
            backLeftLocation.getX(), 
            backLeftLocation.getY()
        )
    );
    public static final Distance backRightDistance = Meter.of(
        Math.hypot(
            backRightLocation.getX(), 
            backRightLocation.getY()
        )
    );

    public static final Rotation2d frontLeftAngle = frontLeftLocation.getAngle();
    public static final Rotation2d frontRightAngle = frontRightLocation.getAngle();
    public static final Rotation2d backLeftAngle = backLeftLocation.getAngle();
    public static final Rotation2d backRightAngle = backRightLocation.getAngle();

    public static final SwerveModulePosition frontLeftPosition = new SwerveModulePosition(frontLeftDistance, frontLeftAngle);
    public static final SwerveModulePosition frontRightPosition = new SwerveModulePosition(frontRightDistance, frontRightAngle);
    public static final SwerveModulePosition backLeftPosition = new SwerveModulePosition(backLeftDistance, backLeftAngle);
    public static final SwerveModulePosition backRightPosition = new SwerveModulePosition(backRightDistance, backRightAngle);

    public static final PIDController velocityPID = new PIDController(1.0, 0.0, 0.0);
    public static final PIDController anglePID = new PIDController(1.0, 0.0, 0.0);
    public static double driveGearRatio = 4.71;
    public static double angleGearRatio = 15.43;
    public static MomentOfInertia driveMoi = 17.493;
    public static MomentOfInertia angleMoi = 0;
    public static Voltage angleFrictionVoltage = Volts.of(0);
    public static Voltage driveFrictionVoltage = Volts.of(0);
    public static boolean absoluteEncoderInverted;
    public static FeedForwardConfig driveFeedForwards;
    public static boolean driveMotorInverted;

    public static final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        0,
        0,
        0
    );
    public static final SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(
        0,
        0,
        0
    );
}
