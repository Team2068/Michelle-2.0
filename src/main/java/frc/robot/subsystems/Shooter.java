// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // TODO: find ids
  SparkMax shooter = new SparkMax(0, MotorType.kBrushless);
  TalonFX pivot = new TalonFX(0, "rio");

  SparkMaxConfig shooterConfig = new SparkMaxConfig();
  TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

  public Shooter() {

    // TODO: Find PID and soft limits for pivot
    pivotConfig.Slot0.kP = 0.3;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.1;
    pivotConfig.Slot0.kG = 0;

    pivotConfig.SoftwareLimitSwitch
    .withForwardSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(160.0)
    .withReverseSoftLimitEnable(true)
    .withReverseSoftLimitThreshold(0.0);
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivot.getConfigurator().apply(pivotConfig);

    shooterConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    shooter.configure(shooterConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
  }

  public void shootSpeed(double speed) {
    shooter.set(speed);
  }

  public void shootVoltage(double volts) {
    shooter.setVoltage(volts);
  }

  public void stopShooter() {
    shooter.stopMotor();
  }

  public void pivotSpeed(double speed) {
    pivot.set(speed);
  }

  public void pivotVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}