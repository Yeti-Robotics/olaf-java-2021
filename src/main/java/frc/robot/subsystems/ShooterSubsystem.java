package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.SparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Limelight;
private double distance;
public enum ShooterStatus{
    FORWARDS, BACKWARDS, OFF
}
public class ShooterSubsystem extends SubsystemBase {
    // ball go brrr
    private WPI_TalonFX launchMotor1;
    private WPI_TalonFX launchMotor2;
    // hood go brrr
    public SparkMax pitchMax;
    // turret go brrr
    public SparkMax slewMax;

    public static ShooterStatus shooterStatus;
    public ShooterSubsystem() {
    launchMotor1 = new WPI_TalonFX(Constants.LAUNCH_DRIVER_1);
    launchMotor2 = new WPI_TalonFX(Constants.LAUNCH_DRIVER_2);

    }
    
}
