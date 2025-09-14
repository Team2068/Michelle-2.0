package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Util;

public class Elevator extends SubsystemBase {
  // TODO: assign correct IDs
  SparkMax elevator = new SparkMax(0, MotorType.kBrushless);
  SparkMax follower = new SparkMax(0, MotorType.kBrushless);

  SparkMaxConfig config = new SparkMaxConfig();
  SparkMaxConfig followerConfig = new SparkMaxConfig();

  PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  double target = 0.0;
  Timer time = new Timer();
  boolean stopped = true;
  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));

  // TODO: Find heights
  public static final double L1 = (double) Util.get("L1 Height", 25.0);
  public static final double L2 = (double) Util.get("L2 Height", 43.5);
  public static final double L3 = (double) Util.get("L3 Height", 76.0);
  public static final double L4 = (double) Util.get("L4 Height", 110.0);


  public Elevator() {
    config.softLimit.forwardSoftLimitEnabled(false);
    config.softLimit.forwardSoftLimit(0); // TODO: Find the Forward soft limit
    config.softLimit.reverseSoftLimitEnabled(false);
    config.softLimit.reverseSoftLimit(0); // TODO: Find the Reverse soft limit
    config.idleMode(SparkMaxConfig.IdleMode.kBrake);

    config.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0); // TODO: Find PID values

    elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerConfig.follow(elevator.getDeviceId());
    followerConfig.inverted(true); // TODO: See if we need to invert the follower
    follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void speed(double speed){
    elevator.set(speed);
  }

  public void voltage(double voltage){
    elevator.setVoltage(voltage);
  }

  public void move(double height){
    stopped = false;
    target = height;
    time.reset();
  }

  public void stop(){
    stopped = true;
    elevator.stopMotor();
  }

  public double position() {
    return elevator.getEncoder().getPosition();
  }

  public double velocity() {
    return elevator.getEncoder().getVelocity();
  }

  public double voltage() {
    return elevator.getAppliedOutput();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", position());

    double cTime = time.get();

    if (stopped)
      return;

    State out = profile.calculate(cTime, new State(L2, velocity()), new State(target, 0));
    elevator.getClosedLoopController().setReference(target, ControlType.kPosition);

    stopped = profile.isFinished(cTime);

    SmartDashboard.putNumber("Elevator Motor Voltage", voltage());

    SmartDashboard.putNumber("Elevator Target Height", target);
    SmartDashboard.putNumber("Elevator cTarget Height", out.position);

    SmartDashboard.putNumber("Elevator Velocity", velocity());
    SmartDashboard.putNumber("Elevator cTarget Velocity", out.velocity);
  }
}