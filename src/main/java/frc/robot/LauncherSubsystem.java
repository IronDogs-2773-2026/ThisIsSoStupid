// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
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
  private final SparkClosedLoopController shooterPidController;
  private final SparkMaxConfig shooterConfig;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    intakeMotor = new SparkMax(Constants.LauncherConstants.kIntakeMotor, SparkMax.MotorType.kBrushless);
    shooterMotor = new SparkMax(Constants.LauncherConstants.kFlywheelMotor, SparkMax.MotorType.kBrushless);
    indexerMotor = new PWMSparkMax(Constants.LauncherConstants.kIndexerMotor);
    shooterPidController = shooterMotor.getClosedLoopController();
    shooterConfig = new SparkMaxConfig();
    // shooterConfig.smartCurrentLimit(35);
    shooterConfig.closedLoop
        .p(Constants.LauncherConstants.kP)
        .i(Constants.LauncherConstants.kI)
        .d(Constants.LauncherConstants.kD)
        .velocityFF(Constants.LauncherConstants.kFF)
        .outputRange(-0.7, 0.7);
    shooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setShooterSpeed(double RPM) {
    shooterPidController.setReference(RPM, SparkMax.ControlType.kVelocity);
  }

  public void setFlywheelDirect(double speed) {
    shooterMotor.set(speed);
  }

  public void setIndexSpeed(double speed) {
    indexerMotor.set(speed);
  }

  double p = Constants.LauncherConstants.kP;
  public void increaseP() {
    p += 0.00001;
    shooterConfig.closedLoop.p(p);
    shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void decreaseP() {
    p -= 0.00001;
    shooterConfig.closedLoop.p(p);
    shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  double ff = Constants.LauncherConstants.kFF;
  public void increaseFF() {
    ff += 0.00001;
    shooterConfig.closedLoop.velocityFF(ff);
    shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void decreaseFF() {
    ff -= 0.00001;
    shooterConfig.closedLoop.velocityFF(ff);
    shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Set Speed", shooterMotor.get());
    SmartDashboard.putNumber("True RPM", shooterMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Shooter P", p);
    // SmartDashboard.putNumber("Shooter FF", ff);
  }
}
