package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    // ball go brrr
    private WPI_TalonFX flyWheel1;
    private WPI_TalonFX flyWheel2;

    public enum ShooterStatus {
        FORWARDS, BACKWARDS, OFF;
    }

    public static ShooterStatus shooterStatus;

    public ShooterSubsystem() {
        flyWheel1 = new WPI_TalonFX(ShooterConstants.FLYWHEEL_1);
        flyWheel2 = new WPI_TalonFX(ShooterConstants.FLYWHEEL_2);

        flyWheel1.setInverted(true);
        flyWheel1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        flyWheel2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    
    }

    public void shoot() {
        flyWheel1.set(ControlMode.PercentOutput, ShooterConstants.SHOOT_1_SPEED);
        flyWheel2.set(ControlMode.PercentOutput, ShooterConstants.SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.FORWARDS;
    }

    public void reverseShoot() {
        flyWheel1.set(ControlMode.PercentOutput, ShooterConstants.REVERSE_SHOOT_1_SPEED);
        flyWheel2.set(ControlMode.PercentOutput, ShooterConstants.REVERSE_SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void stopShoot() {
        flyWheel1.set(ControlMode.PercentOutput, 0);
        flyWheel2.set(ControlMode.PercentOutput, 0);
        shooterStatus = ShooterStatus.OFF;
    }

    //get encoder value methods probably wrong, need review 
    public double getLeftEncoder() {
        return flyWheel1.getSelectedSensorVelocity();
    }

    public double getRightEncoder() {
        return flyWheel2.getSelectedSensorVelocity();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public static ShooterStatus getShooterStatus() {
        return shooterStatus;
    }

    public double getSpeed() {
        return flyWheel1.getMotorOutputPercent();
    }  
    
}
