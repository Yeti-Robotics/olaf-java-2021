// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

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

    public static final double HIGH_GEAR_RATIO = 5.533243;// fudge values  // 6.86;// JVN Values
    public static final double LOW_GEAR_RATIO = 8.01801; // fudge values // 9.93;// JVN Values
    public static final double DISTANCE_PER_PULSE = (3.875 * Math.PI ) / 2048; //wheel diam in inches & falcon CPR
    // placeholder values
    public static final int[] SHIFTER_SOLENOID = {1,6}; 
    public static final int GYRO_ID = 13;

    public static final double ksVolts = 0.583;
    public static final double kvVoltSecondsPerMeter = 0.142;
    public static final double kaVoltSecondsSquaredPerMeter = 0.00871;
    public static final double trackWidthMeters = 1.9730585666928624; //quetisonably meters lol

    public static final double MAX_SPEED_INCHES_PER_SEC = 9.08 * 12.0;
    public static final double MAX_ACCEL_INCHES_PER_SEC2 = 7.5 * 12.0;
  }

  public static final class AutoConstants {
    // note: copy pasted directly
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    //driver station constants
    public static final int RIGHT_JOYSTICK = 1;
    public static final int LEFT_JOYSTICK = 0;
    public static final int SECONDARY_JOYSTICK = 2;
    public static final int DRIVER_STATION_JOY = 0;
    public static final int XBOX_PORT = 1; 
    public static final double TRIGGER_THRESHOLD = 0.75;
  }

  public static final class LEDConstants {
    public static final int ADDRESSABLE_LED = 9;
    public static final int LED_COUNT = 69;
  }

  public static final class IntakeConstants{
    public static final int INTAKE_VICTOR = 8;
    public static final int[] INTAKE_PISTONS_SOLENOID = {0, 7};
    public static final double ROLL_IN_SPEED = 1.0;
    public static final double ROLL_OUT_SPEED = -1.0;
  }

  public static final class HopperConstants{
    public static final int HOPPER_VICTOR = 9;
    public static final double HOPPER_IN_SPEED = .4;
    public static final double HOPPER_OUT_SPEED = -.4; 
  }

  public static final class TurretConstants{
    public static final int TURRET_SPARK = 10;
    
    public static final double COUNTS_PER_REVOLUTION = 42.0;
    public static final double TURRET_GEAR_RATIO = 227.5 / 1.0; //325.0/1.0;
    public static final double COUNTS_PER_DEGREE = (TURRET_GEAR_RATIO) / 360.0;
    public static final double TURRET_ANGLE_THRESHOLD = 5.0;
    public static final double TURRET_MIN_ANGLE = 0.0;
    public static final double TURRET_MAX_ANGLE = 270; // Double check angle. Is estimate
    public static final double TURRET_SPEED = 0.6; // was 0.2
    //placeholders
    public static final double kPTurretVel = 0.01; 
    public static final double kITurretVel = 0;
    public static final double kDTurretVel = 0;
  }
  
  public static final class ShooterConstants{
    public static final int RIGHT_FLYWHEEL = 5; //rightside falcon
    public static final int LEFT_FLYWHEEL = 6; //leftside falcon
    public static final double SHOOT_1_SPEED = 1.0;
    public static final double SHOOT_2_SPEED = 1.0;
    public static final double REVERSE_SHOOT_1_SPEED = -1.0;
    public static final double REVERSE_SHOOT_2_SPEED = -1.0;

    public static final double ENCODER_RESOLUTION = 2048.0;
    public static final double PULLEY_RATIO = 48.0 / 36.0;
    public static final double ENCODER_TIME_CONVERSION = 600.0; // minutes per 100 ms
    public static final double MAX_RPM = 7500;
    public static final double FEED_FORWARD = 69.420; // place
    public static final double RPM_TOLERANCE = 25;
  }


  public static final class HoodConstants{
    public static final int HOOD_SPARK = 11;
    public static final double COUNTS_PER_REVOLUTION = 42.0;
    public static final double HOOD_GEAR_RATIO = 340.0;//510.0/1.0;
    public static final double COUNTS_PER_DEGREE = (HOOD_GEAR_RATIO) / 360.0;
    public static final double HOOD_ANGLE_TOLERANCE = .25;
    public static final double MAX_HOOD_ANGLE = 30.0;
    public static final double HOOD_SPEED = .1;
  }
  
  public static final class PinchRollerConstants{
    public static final int PINCH_ROLLER_VICTOR = 7;
    public static final double PINCH_ROLLER_IN_SPEED = .3;
    public static final double PINCH_ROLLER_OUT_SPEED = -.3;
  }

  public static final class CalcConstants{
    //distance calc constants
    public static final double KNOWN_DISTANCE = 161.3; //inches
    public static final int PIXEL_WIDTH_KNOWN = 65; //pixels
    public static final double KNOWN_TAPE_BOUND_WIDTH = 39.25; //inches
    public static final double FOCAL_LENGTH = ( KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH;
    //trajectory constants
    public static final double GRAVITY = 386.09; // inches/ sec ^2
    public static final double SHOOTER_HEIGHT = 21.5; // inches
  }

  public static final double USER_SPEED_CONTROL_MODIFIER = 0.5; //0.8 in og code
}
