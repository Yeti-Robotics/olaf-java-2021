package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    // ball go brrr
    private WPI_TalonFX rightFlywheel;
    private WPI_TalonFX leftFlywheel;

    public enum ShooterStatus {
        FORWARDS, BACKWARDS, OFF;
    }

    public static ShooterStatus shooterStatus;

    public ShooterSubsystem() {
        rightFlywheel = new WPI_TalonFX(ShooterConstants.RIGHT_FLYWHEEL);
        leftFlywheel = new WPI_TalonFX(ShooterConstants.LEFT_FLYWHEEL);

        rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        rightFlywheel.follow(leftFlywheel);
        rightFlywheel.setInverted(InvertType.OpposeMaster);
    
        shooterStatus = ShooterStatus.OFF;
    }

    public void shootFlywheel() {
        leftFlywheel.set(ControlMode.PercentOutput, ShooterConstants.SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.FORWARDS;
    }

    public void reverseFlywheel() {
        leftFlywheel.set(ControlMode.PercentOutput, ShooterConstants.REVERSE_SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void stopFlywheel() {
        leftFlywheel.set(ControlMode.PercentOutput, 0);
        shooterStatus = ShooterStatus.OFF;
    }

    public double getLeftEncoder() {
        return leftFlywheel.getSelectedSensorVelocity();
    }

    public double getRightEncoder() {
        return rightFlywheel.getSelectedSensorVelocity();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public static ShooterStatus getShooterStatus() {
        return shooterStatus;
    }

    public double getSpeed() {
        return leftFlywheel.getMotorOutputPercent();
    }
}
