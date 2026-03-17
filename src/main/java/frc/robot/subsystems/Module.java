package frc.robot.subsystems;

import org.ironmaple.simulation.motorsims.SimMotorConfigs;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveDriveConstants;

public class Module {
    private SparkFlex driveMotor;
    private SparkFlexConfig driveConfig = new SparkFlexConfig();
    {
        driveConfig
            .absoluteEncoder
                .inverted(SwerveDriveConstants.absoluteEncoderInverted);
        driveConfig
            .closedLoop
                .feedForward
                    .apply(SwerveDriveConstants.driveFeedForwards);
        driveConfig
            .closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(SwerveDriveConstants.velocityPID.getP())
                .i(SwerveDriveConstants.velocityPID.getI())
                .d(SwerveDriveConstants.velocityPID.getD());
        driveConfig
            .inverted(SwerveDriveConstants.driveMotorInverted);
    }
    private SparkMax angleMotor;
    private SparkFlexConfig angleConfig = new SparkFlexConfig();
    {
        angleConfig
            .absoluteEncoder
                .positionConversionFactor(360)
                .inverted(SwerveDriveConstants.absoluteEncoderInverted);
        angleConfig
            .closedLoop
                .feedForward
                    .kA(SwerveDriveConstants.angleFeedForwards.getkA())
                    .kS(0)
                    .kV(0);
        angleConfig
            .closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(SwerveDriveConstants.anglePID.getP())
                .i(SwerveDriveConstants.anglePID.getI())
                .d(SwerveDriveConstants.anglePID.getD());
    }
    private SimMotorConfigs driveMotorConfigs;
    private SimMotorConfigs angleMotorConfigs;

    public Module(int driveMotorID, int angleMotorID) {
        driveMotor = new SparkFlex(driveMotorID, SparkFlex.MotorType.kBrushless);
        angleMotor = new SparkMax(angleMotorID, SparkMax.MotorType.kBrushless);
        if (Robot.isSimulation()) {
            driveMotorConfigs = new SimMotorConfigs(DCMotor.getNeoVortex(1), SwerveDriveConstants.driveGearRatio, SwerveDriveConstants.driveMoi, SwerveDriveConstants.driveFrictionVoltage);
            angleMotorConfigs = new SimMotorConfigs(DCMotor.getNeoVortex(1), SwerveDriveConstants.angleGearRatio, SwerveDriveConstants.angleMoi, SwerveDriveConstants.angleFrictionVoltage);
        }
    }

    public void setState(SwerveModuleState state) {
        // Convert desired state to motor outputs
        double driveOutput = state.speedMetersPerSecond / SwerveDriveConstants.maxLinearVelocity.magnitude();
        double angleOutput = state.angle.getRadians() / (2 * Math.PI); // Normalize to [0, 1]

        // Set motor outputs
        driveMotor.set(driveOutput);
        angleMotor.set(angleOutput);
    }
}
