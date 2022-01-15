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

//The KS Value on Sysid
public static final double ksVolts = 0.65952;
//The Kv Value on Sysid
public static final double kvVoltSecondsPerInch = 0.089694;
//The Ka Value on Sysid
public static final double kaVoltSecondsSquaredPerInch = 0.0054003;

//The Kp Value on Sysid
public static final double kPDriveVel = 0.2266;


public static final double kTrackWidthInches = 21.5;
public static final DifferentialDriveKinematics kDriveKinematics = 
    new DifferentialDriveKinematics(kTrackWidthInches);

public static final double kMaxSpeedInchesPerSecond = 192;

// Reasonable baseline values for a RAMSETE follower in units of Inches and seconds - Might have to retouch if there is error
public static final double kRamseteB = 78.7402;
public static final double kRamseteZeta = 0.7;
}