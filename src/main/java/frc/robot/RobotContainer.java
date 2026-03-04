// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BasicShootAuto;

public class RobotContainer {
  Command m_teleopCommand;
  Command m_autonomousCommand;
  LauncherSubsystem m_launcherSubsystem;
  DriveSubsystem m_driveSubsystem;
  public RobotContainer() {
    m_launcherSubsystem = Constants.kLauncherSubsystem;
    m_driveSubsystem = Constants.kDriveSubsystem;
    configureBindings();
    m_teleopCommand = new DriveCommand(m_driveSubsystem, m_launcherSubsystem);
    m_autonomousCommand = new BasicShootAuto(m_launcherSubsystem);
  }

  private void configureBindings() {}



  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }
}
