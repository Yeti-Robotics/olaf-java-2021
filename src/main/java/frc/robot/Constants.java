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
        public static final int kLeftMotor1Port = 6;
        public static final int kLeftMotor2Port = 3;
        public static final int kRightMotor1Port = 14;
        public static final int kRightMotor2Port = 7;
    
        public static final double HIGH_GEAR_RATIO = 5.13;
        public static final double DISTANCE_PER_PULSE = (0.098552*Math.PI )/2048; //wheel diam in meters :)
        // public static final int[] kLeftEncoderPorts = new int[] {0, 1};
        // public static final int[] kRightEncoderPorts = new int[] {2, 3};
        // public static final boolean kLeftEncoderReversed = false;
        // public static final boolean kRightEncoderReversed = true;
    
        public static final double kTrackwidthMeters = 0.6096;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        // public static final int kEncoderCPR = 1024;
        // public static final double kWheelDiameterMeters = 0.15;
        // public static final double kEncoderDistancePerPulse =
        //     // Assumes the encoders are directly mounted on the wheel shafts
        //     (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.699;
        public static final double kvVoltSecondsPerMeter = 3.51;
        public static final double kaVoltSecondsSquaredPerMeter = 0.208;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.72;
      }
    //piss n shit
      public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
      }
    
      public static final class AutoConstants {
        // note: copy pasted directly
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
 
      }
      public static final class ShooterConstants {
        //PLACEHOLDER VALUES!
        public static final int LAUNCH_DRIVER_1 = 1;
        public static final int LAUNCH_DRIVER_2 = 1;
      }
}
