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
    public static final int LEFT_FALCON_1 = 3;
    public static final int LEFT_FALCON_2 = 4;
    public static final int RIGHT_FALCON_1 = 2;
    public static final int RIGHT_FALCON_2 = 1;

    public static final double HIGH_GEAR_RATIO = 5.13;
    public static final double DISTANCE_PER_PULSE = (0.098552*Math.PI )/2048; //wheel diam in meters :)
  }
    //piss n shit
  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    //driver station constants
    public static final int RIGHT_JOYSTICK = 1;
    public static final int LEFT_JOYSTICK = 0;
    public static final int SECONDARY_JOYSTICK = 2;
    public static final int DRIVER_STATION_JOY = 0;
  }
    
  public static final class IntakeConstants{
    public static final int INTAKE_TALON = 7;
    public static final int[] INTAKE_PISTONS_SOLENOID = {2, 3};
    public static final double ROLL_IN_SPEED = 1.0;
    public static final double ROLL_OUT_SPEED = -1.0;
    public static final int HOPPER_VICTOR = 1;
    public static final int FUNNEL_IN_SPEED = 1;
    public static final int FUNNEL_OUT_SPEED = -1; 
  }

  public static final class ShooterConstants{
    //PLACEHOLDER VALUES!
    public static final int FLYWHEEL_1 = 5; //rightside falcon
    public static final int FLYWHEEL_2 = 6;
    public static final int HOOD_MAX = 7;
    public static final int TURRET_MAX = 8;
    public static final int SHOOT_1_SPEED = 80;
    public static final int SHOOT_2_SPEED = 80;
    public static final int REVERSE_SHOOT_1_SPEED = 10;
    public static final int REVERSE_SHOOT_2_SPEED = 10;
  }
      
  public static final class CalcConstants{
    //distance calc constants
    public static final double KNOWN_DISTANCE = 161.3; //inches
    public static final int PIXEL_WIDTH_KNOWN = 65; //pixels
    public static final double KNOWN_TAPE_BOUND_WIDTH = 39.25; //inches
    public static final double FOCAL_LENGTH = ( KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH;
    //trajectory constants
    public static final double GRAVITY = 386.09; // inches/ sec ^2
    public static final int SHOOTERHEIGHT = 23; //NOT ACTUAL VALUE
  }
}
