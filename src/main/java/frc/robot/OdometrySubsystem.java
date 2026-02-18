// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem extends SubsystemBase {
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final DriveSubsystem driveSubsystem;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final DifferentialDriveKinematics kinematics;

  public OdometrySubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackwidthMeters);
    this.poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        driveSubsystem.getLeftEncoder().getDistance(),
        driveSubsystem.getRightEncoder().getDistance(),
        new Pose2d()
    );
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }
  
  public void resetPose(Pose2d pose) {
    gyro.reset();
    driveSubsystem.getLeftEncoder().reset();
    driveSubsystem.getRightEncoder().reset();
    poseEstimator.resetPosition(gyro.getRotation2d(), driveSubsystem.getLeftEncoder().getDistance(), driveSubsystem.getRightEncoder().getDistance(), pose);
  }

  public double getX() {
    return getPose().getX();
  }

  public double getY() {
    return getPose().getY();
  }

  public double getHeading() {
    return getRotation().getDegrees();
  }

  public void update() {
    poseEstimator.update(
        gyro.getRotation2d(),
        driveSubsystem.getLeftEncoder().getDistance(),
        driveSubsystem.getRightEncoder().getDistance());
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public DifferentialDrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    update();
  }
}