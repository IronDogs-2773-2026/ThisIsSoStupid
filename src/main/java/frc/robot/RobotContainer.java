// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  Command m_teleopCommand;
  public RobotContainer() {
    configureBindings();
    m_teleopCommand = new DriveCommand(DriveSubsystem.getDriveSubsystem());
  }

  private void configureBindings() {}



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }
}
