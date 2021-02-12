package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Limelight;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
public class ShooterSubsystem extends SubsystemBase {

    //pinchroller go brrr
    private VictorSPX fireController;
    // ball go brrr
    private WPI_TalonFX launchMotor1;
    private WPI_TalonFX launchMotor2;
    // hood go brrr
    public PWMSparkMax pitchMax;
    // turret go brrr
    public PWMSparkMax slewMax;

    private double distance;
    public enum ShooterStatus {
        FORWARDS, BACKWARDS, OFF;
    }

    public static ShooterStatus shooterStatus;
    public ShooterSubsystem() {
        launchMotor1 = new WPI_TalonFX(Constants.LAUNCH_DRIVER_1);
        launchMotor2 = new WPI_TalonFX(Constants.LAUNCH_DRIVER_2);
        pitchMax = new PWMSparkMax(Constants.HOOD_MAX);
        slewMax = new PWMSparkMax(Constants.TURRET_MAX);
        fireController = new VictorSPX(Constants.FIRE_VICTOR);

        launchMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        launchMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void shoot() {
        launchMotor1.set(ControlMode.PercentOutput, Constants.SHOOT_1_SPEED);
        launchMotor2.set(ControlMode.PercentOutput, Constants.SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.FORWARDS;
    }

    public void reverseShoot() {
        launchMotor1.set(ControlMode.PercentOutput, Constants.REVERSE_SHOOT_1_SPEED);
        launchMotor2.set(ControlMode.PercentOutput, Constants.REVERSE_SHOOT_2_SPEED);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void stopShoot() {
        launchMotor1.set(ControlMode.PercentOutput, 0);
        launchMotor2.set(ControlMode.PercentOutput, 0);
        shooterStatus = ShooterStatus.OFF;
    }

    public double getLeftEncoder() {
        return launchMotor1.getSelectedSensorVelocity();
    }

    public double getRightEncoder() {
        return launchMotor2.getSelectedSensorVelocity();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) /2;
    }

    public static ShooterStatus getShooterStatus(){
        return shooterStatus;
    }

    public double calcHoodAngle() {
        return Math.toDegrees(Math.asin( -Constants.GRAVITY * distance) / Constants.SHOOT_1_SPEED);
    }

    public double getSpeed() {
        return launchMotor1.getMotorOutputPercent();
    }    

    @Override
    public void periodic() {
        distance = Limelight.getCalculatedDistance();
    }

    public void setHoodAngle(double angle) {
        //insert fancy math here
    }

    public void moveHood(double power){
        pitchMax.set(power);
    }

    public void stopHood(){
        pitchMax.set(0);
    }

    public void moveTurret(double power){
        slewMax.set(power);
    }

    public void stopTurret(){
        slewMax.set(0);
    }
}