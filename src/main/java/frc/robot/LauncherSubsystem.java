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
  // private final SparkClosedLoopController shooterPidController;
  // private final SparkMaxConfig shooterConfig;
  private final PIDController pidController;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    intakeMotor = new SparkMax(Constants.LauncherConstants.kIntakeMotor, SparkMax.MotorType.kBrushless);
    shooterMotor = new SparkMax(Constants.LauncherConstants.kFlywheelMotor, SparkMax.MotorType.kBrushless);
    indexerMotor = new PWMSparkMax(Constants.LauncherConstants.kIndexerMotor);
    // shooterPidController = shooterMotor.getClosedLoopController();
    
    // shooterConfig = new SparkMaxConfig();
    // // shooterConfig.smartCurrentLimit(35);
    // shooterConfig.closedLoop
    //     .p(Constants.LauncherConstants.kP)
    //     .i(Constants.LauncherConstants.kI)
    //     .d(Constants.LauncherConstants.kD)
    //     .velocityFF(Constants.LauncherConstants.kFF)
    //     .allowedClosedLoopError(100, shooterPidController.getSelectedSlot())
    //     .outputRange(-0.6, 0.75)
    //     // .dFilter(1)
    //     ;
    // shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pidController = new PIDController(Constants.LauncherConstants.nkP, Constants.LauncherConstants.nkI, Constants.LauncherConstants.nkD);
    pidController.setTolerance(100);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  // public void setShooterSpeed(double RPM) {
  //   shooterPidController.setReference(RPM, SparkMax.ControlType.kVelocity);
  // }

  public void setFlywheelDirect(double speed) {
    shooterMotor.set(speed);
  }

  public void setShooterBetter(double RPM) {
    double output = pidController.calculate(shooterMotor.getEncoder().getVelocity(), RPM);
    shooterMotor.set(output);
  }

  public void setIndexSpeed(double speed) {
    indexerMotor.set(speed);
  }

  public void stopAll() {
    shooterMotor.stopMotor();
    shooterMotor.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // double p = Constants.LauncherConstants.kP;
  // public void increaseP() {
  //   p += 0.00001;
  //   shooterConfig.closedLoop.p(p);
  //   shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }

  // public void decreaseP() {
  //   p -= 0.00001;
  //   shooterConfig.closedLoop.p(p);
  //   shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }

  // double ff = Constants.LauncherConstants.kFF;
  // public void increaseFF() {
  //   ff += 0.00001;
  //   shooterConfig.closedLoop.velocityFF(ff);
  //   shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }

  // public void decreaseFF() {
  //   ff -= 0.00001;
  //   shooterConfig.closedLoop.velocityFF(ff);
    // shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Set Speed", shooterMotor.get());
    SmartDashboard.putNumber("True RPM", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber();
    // SmartDashboard.putNumber("Shooter P", p);
    // SmartDashboard.putNumber("Shooter FF", ff);
  }
}
