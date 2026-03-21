package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Robot;

public class RealModule implements SwerveModule {
    private final SparkFlex driveMotor;
    private final SparkFlexConfig driveConfig = new SparkFlexConfig();
    {
        driveConfig
            .encoder
                .countsPerRevolution(SwerveDriveConstants.encoderResolution);
        driveConfig
            .closedLoop
                .feedForward
                    .kA(SwerveDriveConstants.driveFeedforward.getKa())
                    .kS(SwerveDriveConstants.driveFeedforward.getKs())
                    .kV(SwerveDriveConstants.driveFeedforward.getKv());
        driveConfig
            .closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(SwerveDriveConstants.drivePID.getP())
                .i(SwerveDriveConstants.drivePID.getI())
                .d(SwerveDriveConstants.drivePID.getD());
        driveConfig
            .inverted(SwerveDriveConstants.driveMotorInverted);
    }
    private final RelativeEncoder driveEncoder;
    private final SparkMax angleMotor;
    private final SparkFlexConfig angleConfig = new SparkFlexConfig();
    {
        angleConfig
            .absoluteEncoder
                .positionConversionFactor(360)
                .inverted(SwerveDriveConstants.absoluteEncoderInverted);
        angleConfig
            .closedLoop
                .feedForward
                    .kA(SwerveDriveConstants.angleFeedforward.getKa())
                    .kS(SwerveDriveConstants.angleFeedforward.getKs())
                    .kV(SwerveDriveConstants.angleFeedforward.getKv());
        angleConfig
            .closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(SwerveDriveConstants.anglePID.getP())
                .i(SwerveDriveConstants.anglePID.getI())
                .d(SwerveDriveConstants.anglePID.getD());
    }
    private final SparkAbsoluteEncoder angleEncoder;
    private final double chassisAngularOffset;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private final SparkClosedLoopController driveClosedLoopController;
    private final SparkClosedLoopController angleClosedLoopController;
    
    public RealModule(int driveMotorID, int angleMotorID, Rotation2d chassisAngularOffset) {
        driveMotor = new SparkFlex(driveMotorID, SparkFlex.MotorType.kBrushless);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        driveEncoder = driveMotor.getEncoder();
        angleMotor = new SparkMax(angleMotorID, SparkMax.MotorType.kBrushless);
        angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        angleEncoder = angleMotor.getAbsoluteEncoder();
        this.chassisAngularOffset = chassisAngularOffset.getDegrees();

        desiredState.angle = new Rotation2d(angleEncoder.getPosition());
        driveEncoder.setPosition(0);

        driveClosedLoopController = driveMotor.getClosedLoopController();
        angleClosedLoopController = angleMotor.getClosedLoopController();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(angleEncoder.getPosition() - chassisAngularOffset));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(angleEncoder.getPosition() - chassisAngularOffset));
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(angleEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        driveClosedLoopController.setSetpoint( correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity );
        angleClosedLoopController.setSetpoint( correctedDesiredState.angle.getRadians(), ControlType.kPosition );

        this.desiredState = desiredState;
    }

    @Override
    public SwerveModuleSimulation getSimModule() {
        return null;
    }
}
