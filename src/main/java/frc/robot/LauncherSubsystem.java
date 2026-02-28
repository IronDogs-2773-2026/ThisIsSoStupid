// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMax shooterMotor;
  private final PWMSparkMax indexerMotor;
  // private final PIDController pidController;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    intakeMotor = new SparkMax(Constants.LauncherConstants.kIntakeMotor, SparkMax.MotorType.kBrushless);
    shooterMotor = new SparkMax(Constants.LauncherConstants.kFlywheelMotor, SparkMax.MotorType.kBrushless);
    indexerMotor = new PWMSparkMax(Constants.LauncherConstants.kIndexerMotor);

    // pidController = new PIDController(Constants.LauncherConstants.nkP, Constants.LauncherConstants.nkI, Constants.LauncherConstants.nkD);
    // pidController.setTolerance(100);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setShooterDirect(double speed) {
    shooterMotor.set(speed);
  }

  // public void setShooterPid(double RPM) {
  //   double output = pidController.calculate(shooterMotor.getEncoder().getVelocity(), RPM);
  //   shooterMotor.set(output);
  // }

  public void setIndexSpeed(double speed) {
    indexerMotor.set(speed);
  }

  public void stopAll() {
    shooterMotor.stopMotor();
    shooterMotor.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Set Speed", shooterMotor.get());
    SmartDashboard.putNumber("True RPM", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
  }
}
