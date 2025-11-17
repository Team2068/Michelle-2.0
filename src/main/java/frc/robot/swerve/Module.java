package frc.robot.swerve;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Module {
    public final TalonFX drive;
    public final SparkMax steer;
    public final SparkAbsoluteEncoder encoder;
    public SparkMaxConfig steerConfig;

    public static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

    public Module(ShuffleboardLayout tab, int driveID, int steerID) {
        drive = new TalonFX(driveID, "rio");
        steer = new SparkMax(steerID, MotorType.kBrushless);
        encoder = steer.getAbsoluteEncoder();

        steerConfig = new SparkMaxConfig();

        steerConfig
                .smartCurrentLimit(20)
                .idleMode(IdleMode.kBrake)
                .inverted(true);

        steerConfig.encoder
                .positionConversionFactor(Math.PI * STEER_REDUCTION)
                .velocityConversionFactor(Math.PI * STEER_REDUCTION / 60);

        steerConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingMaxInput(SwerveConfig.PI2)
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.2, 0.0, 0.0);

        steerConfig.absoluteEncoder.averageDepth(64);
        steerConfig.absoluteEncoder.inverted(true);
        steerConfig.closedLoop.positionWrappingInputRange(0, 1);
        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.signals.primaryEncoderPositionAlwaysOn(false);
        steerConfig.signals.primaryEncoderPositionPeriodMs(10); // Test how changing period affects swerve
        steerConfig.signals.absoluteEncoderPositionPeriodMs(10);
        steerConfig.signals.absoluteEncoderVelocityPeriodMs(10);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        steer.getEncoder().setPosition(angle());

        drive.getConfigurator().apply(config);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(angle()));
        tab.addDouble("Current Angle", () -> Math.toDegrees(steer.getEncoder().getPosition()));
        tab.addDouble("Angle Difference", () -> Math.toDegrees(angle() - steer.getEncoder().getPosition()));
    }

    public void resetDrivePosition() {
        drive.setPosition(0.0);
    }

    public void syncEncoders() {
        steer.getEncoder().setPosition(angle());
    }

    public void zeroAbsolute() {
        steerConfig.absoluteEncoder.zeroOffset(angle());
        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double drivePosition() {
        return drive.getPosition().getValueAsDouble() * .632 * SwerveConfig.Constants.WHEEL_DIAMETER;
    }

    public LinearVelocity velocity() {
        return MetersPerSecond.of(drive.getVelocity().getValueAsDouble() * SwerveConfig.PI2 * .632 * SwerveConfig.Constants.WHEEL_DIAMETER);
    }

    public Voltage voltage() {
        return drive.getMotorVoltage().getValue();
    }

    public double angle() {
        return (encoder.getPosition() * SwerveConfig.PI2) % SwerveConfig.PI2;
    }

    public double steerVelocity() {
        return encoder.getVelocity();
    }

    public Voltage steerVoltage() {
        return Volts.of(steer.getBusVoltage());
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public void set(double driveVolts, double targetAngle) {
        syncEncoders();

        drive.set(driveVolts);
        steer.getClosedLoopController().setReference(targetAngle, ControlType.kPosition);
    }

    public void setSteer(double steerVolts) {
        syncEncoders();
        drive.set(0);
        steer.set(steerVolts);
    }

}