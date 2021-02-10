// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public WPI_TalonFX leftTalonFX;
  public WPI_TalonFX leftTalonFX2;
  public WPI_TalonFX rightTalonFX;
  public WPI_TalonFX rightTalonFX2;
  public DifferentialDrive steeringWheel;
  /** Creates a new Drivetrain. */
  public Drivetrain() {

  leftTalonFX = new WPI_TalonFX(2020);
  leftTalonFX2 = new WPI_TalonFX(2021);
  rightTalonFX = new WPI_TalonFX(202);
  rightTalonFX2 = new WPI_TalonFX(1202);
  steeringWheel = new DifferentialDrive(leftTalonFX, rightTalonFX);

  }

  public void tankDrive(double leftpower, double rightpower) {
  steeringWheel.tankDrive(leftpower, rightpower);


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
