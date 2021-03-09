package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CalcConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

public class ShooterSubsystem extends SubsystemBase {

    // ball go brrr
    private WPI_TalonFX rightFlywheel;
    private WPI_TalonFX leftFlywheel;

    private double distance;

    public enum ShooterStatus {
        FORWARDS, BACKWARDS, OFF;
    }

    public static ShooterStatus shooterStatus;

    public ShooterSubsystem() {
        rightFlywheel = new WPI_TalonFX(ShooterConstants.RIGHT_FLYWHEEL);
        leftFlywheel = new WPI_TalonFX(ShooterConstants.LEFT_FLYWHEEL);

        // rightFlywheel.setInverted(true);
        rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        rightFlywheel.follow(leftFlywheel);
        rightFlywheel.setInverted(InvertType.OpposeMaster);
    
    }

    public void spin(double speed) {
        // rightFlywheel.set(ControlMode.PercentOutput, speed);
        leftFlywheel.set(ControlMode.PercentOutput, speed);
    }

    public void shoot() {
        // rightFlywheel.set(ControlMode.PercentOutput, ShooterConstants.SHOOT_1_SPEED);
        leftFlywheel.set(ControlMode.PercentOutput, ShooterConstants.SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.FORWARDS;
    }

    public void reverseShoot() {
        // rightFlywheel.set(ControlMode.PercentOutput, ShooterConstants.REVERSE_SHOOT_1_SPEED);
        leftFlywheel.set(ControlMode.PercentOutput, ShooterConstants.REVERSE_SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void stopShoot() {
        // rightFlywheel.set(ControlMode.PercentOutput, 0);
        leftFlywheel.set(ControlMode.PercentOutput, 0);
        shooterStatus = ShooterStatus.OFF;
    }

    //get encoder value methods probably wrong, need review 
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

    
    public double calcHoodAngle(){
        return Math.toDegrees(Math.asin(-CalcConstants.GRAVITY * distance) / ShooterConstants.SHOOT_1_SPEED); //shooter speed is placeholder rn
    }

    @Override
    public void periodic(){
        distance = Limelight.getCalculatedDistance();
    }
}
