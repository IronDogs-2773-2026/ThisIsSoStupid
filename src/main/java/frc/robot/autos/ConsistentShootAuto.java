// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.DriveSubsystem;
import frc.robot.LauncherSubsystem;
import frc.robot.OdometrySubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConsistentShootAuto extends SequentialCommandGroup {
  /** Creates a new ConsistentShootAuto. */
  private final DriveSubsystem driveSub;
  private final OdometrySubsystem odomSub;
  private final LauncherSubsystem launcherSub;
  public ConsistentShootAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      driveSub = Constants.kDriveSubsystem;
      odomSub = Constants.kOdometrySubsystem;
      launcherSub = Constants.kLauncherSubsystem;
    addCommands(
      new DriveBack(driveSub, odomSub),
      new WaitCommand(2),
      new flywheelCommand(launcherSub),
      new WaitCommand(2),
      new ShootCommand(launcherSub)

    );
  }
}
