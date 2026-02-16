// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveSubsystem extends SubsystemBase {
  private final PWMSparkMax driveMotorLeft1;
  private final PWMSparkMax driveMotorLeft2;
  private final PWMSparkMax driveMotorRight1;
  private final PWMSparkMax driveMotorRight2;
  
  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private final DifferentialDrive drive;

  public DriveSubsystem() {
    driveMotorLeft1 = new PWMSparkMax(Constants.DriveConstants.kLeftMotor1Port);
    driveMotorLeft2 = new PWMSparkMax(Constants.DriveConstants.kLeftMotor2Port);
    driveMotorRight1 = new PWMSparkMax(Constants.DriveConstants.kRightMotor1Port);
    driveMotorRight2 = new PWMSparkMax(Constants.DriveConstants.kRightMotor2Port);

    drive = new DifferentialDrive(driveMotorLeft1, driveMotorRight1);

    driveMotorLeft1.addFollower(driveMotorLeft2);
    driveMotorRight1.addFollower(driveMotorRight2);

    leftEncoder = new Encoder(0, 1);
    rightEncoder = new Encoder(2, 3);
  }

  public DifferentialDrive getDrive() {
    return drive;
  }

  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}