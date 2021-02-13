// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  public enum IntakeStatus{
    DOWN, UP
  }
  public static IntakeStatus intakeStatus;

  private final DoubleSolenoid intakePistons;
  private final TalonSRX intakeTalon;
  private final VictorSPX hopperVictor; 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeTalon = new TalonSRX(IntakeConstants.INTAKE_TALON);
    intakePistons = new DoubleSolenoid(IntakeConstants.INTAKE_PISTONS_SOLENOID[0], IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
    hopperVictor = new VictorSPX(IntakeConstants.HOPPER_VICTOR);

    intakeTalon.setInverted(true);
    hopperVictor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend(){
    intakePistons.set(DoubleSolenoid.Value.kForward);
    intakeStatus = IntakeStatus.DOWN;
  }
  public void retract(){
      intakePistons.set(DoubleSolenoid.Value.kReverse);
      intakeStatus = IntakeStatus.UP;
  }
  
  public void intakeIn(){
      intakeTalon.set(ControlMode.PercentOutput, IntakeConstants.ROLL_IN_SPEED);
  }
  public void intakeOut(){
      intakeTalon.set(ControlMode.PercentOutput, IntakeConstants.ROLL_OUT_SPEED);
  }
  public void intakeStop(){
      intakeTalon.set(ControlMode.PercentOutput, 0);
  }
  public static IntakeStatus getIntakePosition(){
      return intakeStatus;
  }

  public void hopperIn() {
    hopperVictor.set(ControlMode.PercentOutput, IntakeConstants.FUNNEL_IN_SPEED);
  }
  public void hopperOut() {
    hopperVictor.set(ControlMode.PercentOutput, IntakeConstants.FUNNEL_OUT_SPEED);
  }
  public void hopperStop() {
    hopperVictor.set(ControlMode.PercentOutput, 0);
  }
}
