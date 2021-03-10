// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int LEFT_FALCON_1 = 1;
    public static final int LEFT_FALCON_2 = 2;
    public static final int RIGHT_FALCON_1 = 3;
    public static final int RIGHT_FALCON_2 = 4;

    public static final double HIGH_GEAR_RATIO = 40/26.0;//i think? pls double check
    public static final double LOW_GEAR_RATIO = 34/32.0; // same as abv
    public static final double DISTANCE_PER_PULSE = (0.098552*Math.PI )/2048; //wheel diam in meters :)
    // placeholder values
    public static final int[] SHIFTER_SOLENOID = {0,1}; 
    public static final int GYRO_ID = 13;
  }

  public static final class AutoConstants {
    // note: copy pasted directly
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    //driver station constants
    public static final int RIGHT_JOYSTICK = 1;
    public static final int LEFT_JOYSTICK = 0;
    public static final int SECONDARY_JOYSTICK = 2;
    public static final int DRIVER_STATION_JOY = 0;
  }

  public static final class LEDConstants {
    public static final int ADDRESSABLE_LED = 9;
    public static final int LED_COUNT = 69;
  }

  public static final class IntakeConstants{
    public static final int INTAKE_VICTOR = 8;
    public static final int[] INTAKE_PISTONS_SOLENOID = {2, 3};
    public static final double ROLL_IN_SPEED = 1.0;
    public static final double ROLL_OUT_SPEED = -1.0;
  }

  public static final class HopperConstants{
    public static final int HOPPER_VICTOR = 9;
    public static final double HOPPER_IN_SPEED = .5;
    public static final double HOPPER_OUT_SPEED = -.5; 
  }

  public static final class TurretConstants{
    public static final int TURRET_SPARK = 10;
    public static final double COUNTS_PER_REVOLUTION = 42;
    public static final double TURRET_GEAR_RATIO = 325/1.0;
    public static final double TURRET_ANGLE_THRESHOLD = 5;
  }
  
  public static final class ShooterConstants{
    //PLACEHOLDER VALUES!
    public static final int RIGHT_FLYWHEEL = 5; //rightside falcon
    public static final int LEFT_FLYWHEEL = 6; //leftside falcon
    public static final int SHOOT_1_SPEED = 80;
    public static final int SHOOT_2_SPEED = 80;
    public static final int REVERSE_SHOOT_1_SPEED = 10;
    public static final int REVERSE_SHOOT_2_SPEED = 10;
  }


  public static final class HoodConstants{
    public static final int HOOD_SPARK = 11;
    public static final double COUNTS_PER_REVOLUTION = 42.0;
    public static final double HOOD_GEAR_RATIO = 510.0/1.0;
    public static final int HOOD_ANGLE_THRESHOLD = 5;
    public static final double FORWARD_SOFT_LIMIT = ((36.0/340.0)*360.0);
  }
  
  public static final class PinchRollerConstants{
    public static final int PINCH_ROLLER_VICTOR = 7;
    public static final double PINCH_ROLLER_IN_SPEED = .75;
    public static final double PINCH_ROLLER_OUT_SPEED = -.75;
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
