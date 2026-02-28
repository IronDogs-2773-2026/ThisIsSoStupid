// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  Command m_teleopCommand;
  LauncherSubsystem m_launcherSubsystem;
  DriveSubsystem m_driveSubsystem;
  OdometrySubsystem m_odomSub;
  public RobotContainer() {
    m_driveSubsystem = Constants.kDriveSubsystem;
    m_launcherSubsystem = Constants.kLauncherSubsystem;
    m_odomSub = Constants.kOdometrySubsystem;
    configureBindings();
    m_teleopCommand = new DriveCommand(m_driveSubsystem, m_launcherSubsystem, m_odomSub);
  }

  private void configureBindings() {}



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }
}
