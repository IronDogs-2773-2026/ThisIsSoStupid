// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final DriveSubsystem driveSub;
  private final LauncherSubsystem launcherSub;
  private final XboxController controllerDrive = new XboxController(0);
  private final XboxController controllerShoot = new XboxController(1);
  private final SlewRateLimiter filter = new SlewRateLimiter(Constants.kBigNumber, -2000.0, 0);
  private final OdometrySubsystem odomSub;

  public DriveCommand(DriveSubsystem driveSub, LauncherSubsystem launcherSub, OdometrySubsystem odomSub) {
    this.driveSub = driveSub;
    this.launcherSub = launcherSub;
    this.odomSub = odomSub;
    addRequirements(driveSub, launcherSub, odomSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSub.stopAll();
    odomSub.resetPose(
        new Pose2d(
        SmartDashboard.getNumber("Set X", 0), 
        SmartDashboard.getNumber("Set Y", 0), 
        new Rotation2d(SmartDashboard.getNumber("Set Angle", 0))));
  }

  // not using pid for this competition, we need to safeguard the motors burning
  // out - maybe only 60% power
  public void shoot() {
    // launcherSub.setShooterDirect(controllerShoot.getLeftTriggerAxis() * 0.7);
    if (controllerShoot.getLeftTriggerAxis() > 0.5) {
      launcherSub.setShooterPid(3000);
    } else {
      launcherSub.setShooterPid(0);
    }

    if (controllerShoot.getRightTriggerAxis() > 0.5) {
      launcherSub.setIndexSpeed(-1);
      launcherSub.setIntakeSpeed(-1);
    } else if (controllerShoot.getRightBumperButton() && controllerShoot.getLeftTriggerAxis() < 0.5) {
      launcherSub.setIndexSpeed(0.6);
      launcherSub.setIntakeSpeed(-1);
    } else if (controllerShoot.getLeftBumperButton() && controllerShoot.getLeftTriggerAxis() < 0.5) {
      launcherSub.setIntakeSpeed(1);
      launcherSub.setIndexSpeed(-1);
      launcherSub.setShooterDirect(-0.2);
    } else {
      launcherSub.setIndexSpeed(0);
      launcherSub.setIntakeSpeed(0);
    }

    if (controllerShoot.getAButton()) {
      launcherSub.setShooterDirect(0.3);
      launcherSub.setIntakeSpeed(-1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.getDrive().curvatureDrive(controllerDrive.getRightX() * Constants.DriveConstants.kSensitivity,
        -controllerDrive.getLeftY() * Constants.DriveConstants.kSensitivity, true);
    shoot();
    // tunePID();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSub.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
