// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // TODO: find ids
   SparkMax shooter = new SparkMax(0, MotorType.kBrushless);
   SparkMax pivot = new SparkMax(0, MotorType.kBrushless);

   SparkMaxConfig shooterConfig = new SparkMaxConfig();
   SparkMaxConfig pivotConfig = new SparkMaxConfig();


  public Shooter() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
