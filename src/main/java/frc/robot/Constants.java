// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Constants {

    public class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final double kTrackwidthMeters = 0.69;
    }

    public static final int kLiftMotorPort = 29;

    public class LauncherConstants {
        public static final int kFlywheelMotorPort1 = 6;        
        public static final int kFlywheelMotorPort2 = 7;
        public static final int kSlurpyBallsMotorPort = 24;

    }

    public static final XboxController controller = new XboxController(0);
}