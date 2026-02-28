// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Constants {
    public static final DriveSubsystem kDriveSubsystem = new DriveSubsystem();
    public static final LauncherSubsystem kLauncherSubsystem = new LauncherSubsystem();
    public static final OdometrySubsystem kOdometrySubsystem = new OdometrySubsystem(kDriveSubsystem);
    public class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;
        public static final double kTrackwidthMeters = 0.69;
        public static final double kSensitivity = 0.6;
    }

    public static final int kLiftMotorPort = 29;
    public static final double kBigNumber = 999999;

    public class LauncherConstants {
        public static final int kIntakeMotor = 14;        
        public static final int kIndexerMotor = 7;
        public static final int kFlywheelMotor = 32;
        // public static final double kP = 0.00185;
        // public static final double kI = 0.0000000138;
        // public static final double kD = 0.00000001;      
        // public static final double kFF = 0.000176 * 2.5;

        public static final double nkP = 0.00001;
        public static final double nkI = 0.0;
        public static final double nkD = 0.0 ; // 0.000002;      
    }

    public static final XboxController controller = new XboxController(0);
}