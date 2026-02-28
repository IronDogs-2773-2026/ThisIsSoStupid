// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.OdometrySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveBack extends Command {
  private final DriveSubsystem driveSub;
  private final OdometrySubsystem odomSub;
  double difference;
  Pose2d oldPose = null;
  /** Creates a new DriveBack. */
  public DriveBack(DriveSubsystem driveSub, OdometrySubsystem odomSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
    this.odomSub = odomSub;

    addRequirements(driveSub, odomSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
      double heading = odomSub.getHeading();
      Pose2d currentPose = odomSub.getPose();
      if(difference < 2) {
        if(oldPose != null) {
        difference = currentPose.getTranslation().getDistance(oldPose.getTranslation());
        }
        oldPose = currentPose;
      if (heading >= 0 && heading <= 45) {
        driveSub.getDrive().arcadeDrive(-0.5, 0);
      } else if (heading >= 45 && heading <= 135) {
        driveSub.getDrive().arcadeDrive(-0.5, -0.5);
      } else if (heading >= 135 && heading <= 225) {
        driveSub.getDrive().arcadeDrive(-0.5, 0.5);
      } else if (heading >= 225 && heading <= 315) {
        driveSub.getDrive().arcadeDrive(-0.5, 0);
      } else {
        driveSub.getDrive().arcadeDrive(0, 0);
      }
    }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
