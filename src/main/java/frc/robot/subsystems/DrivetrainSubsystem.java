package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class DrivetrainSubsystem extends SubsystemBase {
  
  private WPI_TalonFX leftFalcon1, leftFalcon2, rightFalcon1, rightFalcon2;  

  private PigeonIMU gyro;
  
  // The robot's drive
  public final DifferentialDrive m_drive;
  private SpeedControllerGroup m_leftMotors;
  private SpeedControllerGroup m_rightMotors; 

  // converts desired linear & angular velocities to desired velocities 
  // for the left & right sides of the drivetrain
  private DifferentialDriveKinematics kinematics;

  // tracks robot pose (where it is on the field) using gyro & encoder values
  private DifferentialDriveOdometry odometry; 

  private DifferentialDriveWheelSpeeds wheelSpeeds; 
  private Pose2d pose; 
  private SimpleMotorFeedforward feedforward;

  private PIDController leftPIDController;
  private PIDController rightPIDController;

  private DriveMode driveMode;

  public enum DriveMode {
    TANK, CHEEZY, ARCADE;
  }

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {   
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    leftFalcon2.follow(leftFalcon1);
    leftFalcon2.setInverted(InvertType.FollowMaster);
    rightFalcon2.follow(rightFalcon1);
    rightFalcon2.setInverted(InvertType.FollowMaster);
    
    gyro = new PigeonIMU(DriveConstants.GYRO_ID);

    kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidthMeters);
    odometry = new DifferentialDriveOdometry(getHeading());
    feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
    wheelSpeeds = new DifferentialDriveWheelSpeeds();

    // only need P
    leftPIDController = new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0);
    rightPIDController = new PIDController(DriveConstants.kPDriveVel, 0.0, 0.0);

    m_leftMotors = new SpeedControllerGroup(leftFalcon1, leftFalcon2);
    m_rightMotors = new SpeedControllerGroup(rightFalcon1, rightFalcon2);
    m_drive = new DifferentialDrive(leftFalcon1, rightFalcon1);
    m_drive.setDeadband(0.05);

    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    leftFalcon1.setNeutralMode(NeutralMode.Brake);
    rightFalcon1.setNeutralMode(NeutralMode.Brake);
    resetEncoders();

    driveMode = DriveMode.CHEEZY;
  }

  @Override
  public void periodic(){
    wheelSpeeds.leftMetersPerSecond = getMetersPerSecondFromEncoder(leftFalcon1.getSelectedSensorVelocity()); 
    wheelSpeeds.rightMetersPerSecond = getMetersPerSecondFromEncoder(rightFalcon1.getSelectedSensorVelocity()); 
    // update pose using gyro and encoder values
    pose = odometry.update(getHeading(), wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  // BiConsumer is essentially a hacky way to have a mini, two input void method
  // that can be passed as a method reference; RameseteCommand requires it
  public BiConsumer getDifferentialDriveConsumer(){
    BiConsumer<Double, Double> output = (leftVolts, rightVolts) -> tankDriveVolts(leftVolts, rightVolts); 
    return output;
  }

  // public void setLeftVolts(double leftVolts){
  //   // divide by 12.0 to convert volts (0-12) to a percentage
  //   leftFalcon1.set(ControlMode.PercentOutput, leftVolts / 12.0);
  // }

  // public void setRightVolts(double rightVolts){
  //   // divide by 12.0 to convert volts (0-12) to a percentage
  //   rightFalcon1.set(ControlMode.PercentOutput, rightVolts / 12.0);
  // }

  public void tankDrive(double leftpower, double rightpower) {
    m_drive.tankDrive(leftpower, rightpower);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    // might have to invert these is setInvert doesn't work
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void cheezyDrive(double straight, double turn) {
    m_drive.curvatureDrive(straight, -turn, false);
  }

  public void arcadeDrive(double straight, double turn) {
    m_drive.arcadeDrive(straight, -turn);
  }

  public void stopDrive() {
    leftFalcon1.set(ControlMode.PercentOutput, 0);
    rightFalcon1.set(ControlMode.PercentOutput, 0);
  }
    
  public void resetEncoders() {
    leftFalcon1.setSelectedSensorPosition(0);
    rightFalcon1.setSelectedSensorPosition(0);
  }

  public double getLeftEncoder() {
    return (leftFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE)  / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoder() {
    return (- rightFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getAverageEncoder(){
    return ((getLeftEncoder()+getRightEncoder())/2);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  
  public void resetGyro(){
    gyro.setYaw(0);
  }
  
  public double getRawEncoder() {
    return leftFalcon1.getSelectedSensorPosition(); //temp method
  }
  
  public DriveMode getDriveMode(){
    return driveMode;
  }
  
  public void setDriveMode(DriveMode driveMode){
    this.driveMode = driveMode;
  }

  public DifferentialDriveWheelSpeeds getDifferentialDriveSpeeds(){
    return wheelSpeeds;
  }
  
  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getPose(){
    return pose;
  }

  /*
    takes raw falcon encoder value per 100ms and returns meters per second
    of the drivetrain wheels

    10.0 = 100 ms to 1s
    2.0 * PI * r = circumference of the wheel; rotations -> meters traveled
    2048.0 = CPR of falcon encoders
  */
  private double getMetersPerSecondFromEncoder(double raw){
    double ratio = (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO);
    return (10.0 * raw * 2.0 * Math.PI * Units.inchesToMeters(DriveConstants.WHEEL_RADIUS)) / (2048.0 * ratio);
  }
  
  public double getAngle(){
    double [] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  private Rotation2d getHeading(){
    // negative applied because gType safety: The expression of type BiConsumer needs unchecked conversion to conform to BiConsumer<Double,Double>Java(16777748)yro (presumably) returns positive degrees 
    // as the gyro turns ccw; we want the opposite, as the opposite is true 
    // in math / on the unit circle
    return Rotation2d.fromDegrees(-getAngle()); 
  }
}
