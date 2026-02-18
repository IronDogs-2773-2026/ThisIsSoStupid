// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  private final SparkMax flywheelMotor1;
  private final SparkMax flywheelMotor2;
  private final SparkMax slurpyMotor;
  private final MotorControllerGroup flywheelMotors;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    flywheelMotor1 = new SparkMax(Constants.LauncherConstants.kFlywheelMotorPort1, SparkMax.MotorType.kBrushed);
    flywheelMotor2 = new SparkMax(Constants.LauncherConstants.kFlywheelMotorPort2, SparkMax.MotorType.kBrushed);
    slurpyMotor = new SparkMax(Constants.LauncherConstants.kSlurpyBallsMotorPort, SparkMax.MotorType.kBrushless);

    flywheelMotor2.setInverted(true);
    flywheelMotors = new MotorControllerGroup(flywheelMotor1, flywheelMotor2);
  }

  public void setFlywheelSpeed(double speed) {
    flywheelMotors.set(speed);
  }

  public void setSlurpySpeed(double speed) {
    slurpyMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
