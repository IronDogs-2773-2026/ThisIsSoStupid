// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final DriveSubsystem driveSub;
  private final LauncherSubsystem launcherSub;
  private final XboxController controllerDrive = new XboxController(0);
  private final XboxController controllerShoot = new XboxController(1);

  public DriveCommand(DriveSubsystem driveSub, LauncherSubsystem launcherSub) {
    this.driveSub = driveSub;
    this.launcherSub = launcherSub;
    addRequirements(driveSub, launcherSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public void intake() {
    if (controllerShoot.getLeftTriggerAxis() > 0.5) {

      launcherSub.setShooterSpeed(4000);
    } else {
      launcherSub.setShooterSpeed(0);
    }

    if (controllerShoot.getRightTriggerAxis() > 0.5) {
      launcherSub.setIndexSpeed(1);
      launcherSub.setIntakeSpeed(1);
    } else if (controllerShoot.getRightBumperButton()) {
      launcherSub.setIndexSpeed(-1);
      launcherSub.setIntakeSpeed(-1);
    } else {
      launcherSub.setIndexSpeed(0);
      launcherSub.setIntakeSpeed(0);
    }

    if (controllerShoot.getAButton()) {
      launcherSub.setShooterSpeed(2500);
      launcherSub.setIntakeSpeed(1);

    }
  }

  public void shoot() {
    if (controllerShoot.getRightTriggerAxis() > 0.5) {
      launcherSub.setShooterSpeed(4000);
    } else {
      launcherSub.setShooterSpeed(0);
    }
    if (controllerShoot.getLeftTriggerAxis() > 0.5 && !controllerShoot.getLeftBumperButton()) {
      launcherSub.setIndexSpeed(1);
    } else if (controllerShoot.getLeftBumperButton()) {
      launcherSub.setIndexSpeed(-1);
    } else {
      launcherSub.setIndexSpeed(0);
    }

    if (controllerShoot.getAButton()) {
      launcherSub.setIntakeSpeed(1);
    } else if (controllerShoot.getRightBumperButton()) {
      launcherSub.setIntakeSpeed(-1);
    } else {
      launcherSub.setIntakeSpeed(0);
    }
  }

  // public void tunePID() {
  // if (controller.getPOV() == 0) {
  // launcherSub.increaseP();
  // } else if (controller.getPOV() == 180) {
  // launcherSub.decreaseP();
  // } else if (controller.getPOV() == 90) {
  // launcherSub.increaseFF();
  // } else if (controller.getPOV() == 270) {
  // launcherSub.decreaseFF();
  // }
  // }

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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
