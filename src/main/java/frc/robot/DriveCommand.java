// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final DriveSubsystem driveSub;
  private final LauncherSubsystem launcherSub;
  private final XboxController controllerDrive = new XboxController(0);
  private final XboxController controllerShoot = new XboxController(1);
  private final SlewRateLimiter filter = new SlewRateLimiter(Constants.kBigNumber, -2000.0, 0);

  public DriveCommand(DriveSubsystem driveSub, LauncherSubsystem launcherSub) {
    this.driveSub = driveSub;
    this.launcherSub = launcherSub;
    addRequirements(driveSub, launcherSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSub.stopAll();
  }

  // not using pid for this competition, we need to safeguard the motors burning out - maybe only 60% power
  public void shoot() {
    if (controllerShoot.getLeftTriggerAxis() > 0.5) {
      // launcherSub.setShooterSpeed(2773);
      launcherSub.setShooterPid(5500);
      // launcherSub.setShooterDirect(0.6);
    } else if (controllerShoot.getAButton() && controllerShoot.getLeftTriggerAxis() <= 0.5) {
      // launcherSub.setShooterDirect(0.45);
      launcherSub.setShooterPid(4500);
    } else {
      // launcherSub.setShooterSpeed(0);
      launcherSub.setShooterPid(0);
      // launcherSub.setShooterDirect(0);
    }

    if (controllerShoot.getRightTriggerAxis() > 0.5) {
      launcherSub.setIndexSpeed(-0.77);
      launcherSub.setIntakeSpeed(-0.5);
    } else if (controllerShoot.getRightBumperButton() && controllerShoot.getLeftTriggerAxis() < 0.5) {
      launcherSub.setShooterPid(-300);
      launcherSub.setIndexSpeed(0.9);
      launcherSub.setIntakeSpeed(-0.85);
    } else if (controllerShoot.getLeftBumperButton() && controllerShoot.getLeftTriggerAxis() < 0.5) {
      launcherSub.setIntakeSpeed(0.6);
      launcherSub.setIndexSpeed(-0.6);
      launcherSub.setShooterPid(-400);

    } else {
      launcherSub.setIndexSpeed(0);
      launcherSub.setIntakeSpeed(0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double multiplier = (controllerDrive.getRightTriggerAxis() > 0.5) ? 0.6 : 1;
    driveSub.getDrive().curvatureDrive(controllerDrive.getRightX() * Constants.DriveConstants.kSensitivity,
        -controllerDrive.getLeftY() * multiplier, true);
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
