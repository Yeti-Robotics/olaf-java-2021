package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {private final WPI_TalonFX leftfalcon1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
    private final WPI_TalonFX leftfalcon2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
    private final WPI_TalonFX rightfalcon1 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
    private final WPI_TalonFX rightfalcon2 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);
  

  
    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(leftfalcon1, rightfalcon1);
  
 
  
    // The gyro sensor
    private final Gyro gyro = new ADXRS450_Gyro();
  
    
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
  
    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
 
      resetEncoders();
      m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
      gyro.calibrate();
  
      leftfalcon2.follow(leftfalcon1);
      leftfalcon2.setInverted(InvertType.FollowMaster);
      rightfalcon2.follow(rightfalcon2);
      rightfalcon2.setInverted(InvertType.FollowMaster);
  
      m_drive.setDeadband(0.05);
  
      leftfalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
      rightfalcon2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    }
  
    @Override
    public void periodic() {}

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
      m_drive.arcadeDrive(fwd, rot);
    }
  
    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
      leftfalcon1.setVoltage(leftVolts);
      rightfalcon2.setVoltage(-rightVolts);
      m_drive.feed();
    }
  
    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      leftfalcon1.setSelectedSensorPosition(0);
      rightfalcon1.setSelectedSensorPosition(0);
    }
  
    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
      return (getLeftEncoder() + getRightEncoder()) / 2;
    }
  
    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
      return (leftfalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (DriveConstants.HIGH_GEAR_RATIO)) ;
    }
  
    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
      return (rightfalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (DriveConstants.HIGH_GEAR_RATIO)) ;
    }
  
    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
      m_drive.setMaxOutput(maxOutput);
    }
  
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
      gyro.reset();
    }
  
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
      return gyro.getRotation2d().getDegrees();
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
      return -gyro.getRate();
    }
    
  }

    

