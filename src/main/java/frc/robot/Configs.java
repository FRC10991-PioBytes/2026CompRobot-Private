package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.*;


public final class Configs 
{
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                        / ModuleConstants.kDrivingMotorReduction;
                double turningFactor = 2 * Math.PI;
                double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                drivingConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .openLoopRampRate(0.25)
                        .closedLoopRampRate(0.25);
                drivingConfig.encoder
                        .positionConversionFactor(drivingFactor) // meters
                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                drivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.04, 0, 0)
                        //.velocityFF(drivingVelocityFeedForward)
                        .outputRange(-1, 1)
                        .feedForward
                                .kV(drivingVelocityFeedForward);

                turningConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);
                turningConfig.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                        .inverted(true)
                        .positionConversionFactor(turningFactor) // radians
                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                turningConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class Feeder {
        public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();

        static {
                leaderConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12)
                        .closedLoopRampRate(0.5);
                leaderConfig.closedLoop
                        .pid(FeederConstants.kP, 0, FeederConstants.kD)
                        .outputRange(-1, 1)
                        .feedForward
                                .kS(FeederConstants.kStaticFF) // 0.002
                                .kV(FeederConstants.kVelocityFF) // 0.00265
                                .kA(FeederConstants.kAccelerationFF);
                leaderConfig.closedLoop.maxMotion
                        .maxAcceleration(2000);

        }
    }

    public static final class Shooter {
        public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightConfig = new SparkMaxConfig();

        static {
                leaderConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12)
                        .closedLoopRampRate(0.5);
                leaderConfig.closedLoop
                        .pid(ShooterConstants.kP,0, ShooterConstants.kD)
                        .outputRange(-1, 1)
                        .feedForward
                                .kS(ShooterConstants.kStaticFF)
                                .kV(ShooterConstants.kVelocityFF)
                                .kA(ShooterConstants.kAccelerationFF);
                leaderConfig.closedLoop.maxMotion
                        .maxAcceleration(ShooterConstants.kShooterMaxAcceleration);

                rightConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .follow(Constants.ShooterConstants.kLeaderShooterCanId, true);

        }
    }

    public static final class Intake {
        public static final SparkMaxConfig leaderRollerConfig = new SparkMaxConfig();

        public static final SparkMaxConfig leaderPivotConfig = new SparkMaxConfig();

        static {
                // Roller configs
                leaderRollerConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40);

                // Pivot configs
                double turningFactor = 2 * Math.PI;

                leaderPivotConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40);
                leaderPivotConfig.absoluteEncoder
                        .positionConversionFactor(turningFactor * 14.0 / 32.0) // radians with gear ratio
                        .velocityConversionFactor(turningFactor / 60.0 * 14.0 / 32.0); // radians per second wtih gear ratio
                leaderPivotConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(0, 0, 0, ClosedLoopSlot.kSlot0)
                        .outputRange(-1, 1)
                        .positionWrappingEnabled(false)
                        .feedForward
                                .kS(IntakeConstants.kStaticFF)
                                .kV(IntakeConstants.kVelocityFF)
                                .kA(IntakeConstants.kAccelerationFF)
                                .kCos(IntakeConstants.kIntakeGravityCosVoltage)
                                .kCosRatio(1 / turningFactor);
                leaderPivotConfig.closedLoop.maxMotion
                        .cruiseVelocity(Math.PI / 8)
                        .maxAcceleration(Math.PI / 2)
                        .allowedProfileError(0.05);
        }
    }
}