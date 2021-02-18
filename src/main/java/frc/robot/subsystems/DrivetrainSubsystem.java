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
  
  private WPI_TalonFX leftfalcon1, leftfalcon2, rightfalcon1, rightfalcon2;  

  private PigeonIMU lmaogyro;
  
  //so like im gonna create a private SRX idk what this is but allison told me and yea lmao
  private TalonSRX lmaogyrotalon;

  // The robot's drive
  private final DifferentialDrive m_drive;

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {   
    leftfalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftfalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightfalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightfalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    leftfalcon2.follow(leftfalcon1);
    rightfalcon2.follow(rightfalcon1);
    // rightfalcon1.setInverted(true);
    // rightfalcon2.setInverted(true);

    m_drive = new DifferentialDrive(leftfalcon1, rightfalcon1);
    m_drive.setDeadband(0.05);

    leftfalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightfalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    leftfalcon1.setNeutralMode(NeutralMode.Brake);
    rightfalcon1.setNeutralMode(NeutralMode.Brake);
    resetEncoders();
  
    lmaogyrotalon = new TalonSRX(DriveConstants.GYRO_TALON);
    lmaogyro = new PigeonIMU(lmaogyrotalon);
  }

  @Override
  public void periodic(){
    // System.out.println(getAngle());
  }

  public void tankDrive(double fwd, double rot) {
    m_drive.tankDrive(fwd, rot);
  }
  public void stopDrive() {
    leftfalcon1.set(ControlMode.PercentOutput, 0);
    rightfalcon1.set(ControlMode.PercentOutput, 0);
  }
    
  public void resetEncoders() {
    leftfalcon1.setSelectedSensorPosition(0);
    rightfalcon1.setSelectedSensorPosition(0);
  }

  public double getLeftEncoder() {
    return (leftfalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (DriveConstants.HIGH_GEAR_RATIO)) ;
  }

  public double getRightEncoder() {
    return (rightfalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (DriveConstants.HIGH_GEAR_RATIO)) ;
  }

  public double getAverageEncoder(){
    return ((getLeftEncoder()+getRightEncoder())/2);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getAngle(){
    double [] ypr = new double[3];
    lmaogyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void resetGyro(){
    lmaogyro.setYaw(0);
  }

}
