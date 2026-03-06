// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.LauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicShootAuto extends SequentialCommandGroup {
  LauncherSubsystem launcherSub;
  /** Creates a new BasicShootAuto. */
  public BasicShootAuto(LauncherSubsystem launcherSub) {
    this.launcherSub = launcherSub;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> launcherSub.setShooterPid(4500)).withTimeout(1),
      new WaitCommand(1),
      new RunCommand(() -> {
        launcherSub.setIndexSpeed(-0.6);
        launcherSub.setIntakeSpeed(-0.4);
      }).withTimeout(2),
      new WaitCommand(15),
      new RunCommand(() -> {
        launcherSub.stopAll();
      })
    );
  }
}
