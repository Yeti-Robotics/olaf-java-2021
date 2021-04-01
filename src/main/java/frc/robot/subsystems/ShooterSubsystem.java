package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

public class ShooterSubsystem extends SubsystemBase {
    private WPI_TalonFX rightFlywheel;
    private WPI_TalonFX leftFlywheel;
    public double setPoint;

    public enum ShooterStatus {
        FORWARDS, BACKWARDS, OFF;
    }

    public static ShooterStatus shooterStatus;

    public ShooterSubsystem() {
        rightFlywheel = new WPI_TalonFX(ShooterConstants.RIGHT_FLYWHEEL);
        leftFlywheel = new WPI_TalonFX(ShooterConstants.LEFT_FLYWHEEL);

        rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        
        rightFlywheel.follow(leftFlywheel);
        rightFlywheel.setInverted(InvertType.OpposeMaster);
    
        shooterStatus = ShooterStatus.OFF;

        //coast mode
        leftFlywheel.setNeutralMode(NeutralMode.Coast);
        rightFlywheel.setNeutralMode(NeutralMode.Coast);

        //closed loop configuration
        leftFlywheel.configNominalOutputForward(0.0);
        leftFlywheel.configNominalOutputReverse(0.0);

        leftFlywheel.selectProfileSlot(0, 0);
        leftFlywheel.config_kF(0, 0.0517594042839351); // centeres around 16,000 vel/100ms
        leftFlywheel.config_kP(0, 0.25); 
        leftFlywheel.config_kI(0, 0.0000005); //0.0003
        leftFlywheel.config_kD(0, 2.7);//2.5);//1.9);

        setPoint = 6700;
    }
    @Override
    public void periodic(){
    }

    public void shootFlywheel() {
        leftFlywheel.set(ControlMode.Velocity, getVelocityUnitsFromRPM(calcFlywheelRPM()));
        shooterStatus = ShooterStatus.FORWARDS;
    }

    public void shootFlywheel(double speed) {
        leftFlywheel.set(ControlMode.PercentOutput, speed);
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

    public double getSetSpeed() {
        return leftFlywheel.getMotorOutputPercent();
    }

    public double getFlywheelRPM() {
        return getAverageEncoder() * ShooterConstants.PULLEY_RATIO * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION);
    }

    public double getVelocityUnitsFromRPM(double RPM){
        return RPM / (ShooterConstants.PULLEY_RATIO * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION));
    }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public double calcFlywheelRPM(){
        double distance = Limelight.getHorDistance();
        if(distance < 186.0){
            return 6600.0;
        }
        return 6700.0;
    }


}
