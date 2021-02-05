// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
      //placeholder values FIX!
        public static final int kLeftMotor1Port = 6;
        public static final int kLeftMotor2Port = 3;
        public static final int kRightMotor1Port = 14;
        public static final int kRightMotor2Port = 7;
    
    }
    public static final class ShooterConstants {
      //placeholder values FIX!
      public static final int LAUNCH_DRIVER_1 = 1;
      public static final int LAUNCH_DRIVER_2 = 1;

    }
}
