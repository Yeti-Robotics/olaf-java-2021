package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class DrivetrainSubsystem extends SubsystemBase {
  
  private WPI_TalonFX leftFalcon1, leftFalcon2, rightFalcon1, rightFalcon2;  

  private PigeonIMU gyro;
  
  //so like im gonna create a private SRX idk what this is but allison told me and yea lmao
  private TalonSRX gyroTalon;

  // The robot's drive
  private final DifferentialDrive m_drive;

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {   
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    leftFalcon2.follow(leftFalcon1);
    rightFalcon2.follow(rightFalcon1);
    // rightfalcon1.setInverted(true);
    // rightfalcon2.setInverted(true);

    m_drive = new DifferentialDrive(leftFalcon1, rightFalcon1);
    m_drive.setDeadband(0.05);

    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    leftFalcon1.setNeutralMode(NeutralMode.Brake);
    rightFalcon1.setNeutralMode(NeutralMode.Brake);
    resetEncoders();
  
    gyroTalon = new TalonSRX(DriveConstants.GYRO_TALON);
    gyro = new PigeonIMU(gyroTalon);
  }

  @Override
  public void periodic(){
    // System.out.println(getAngle());
  }

  public void tankDrive(double fwd, double rot) {
    m_drive.tankDrive(fwd, rot);
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
    return (leftFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (DriveConstants.HIGH_GEAR_RATIO)) ;
  }

  public double getRightEncoder() {
    return (rightFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (DriveConstants.HIGH_GEAR_RATIO)) ;
  }

  public double getAverageEncoder(){
    return ((getLeftEncoder()+getRightEncoder())/2);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getAngle(){
    double [] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void resetGyro(){
    gyro.setYaw(0);
  }

}
