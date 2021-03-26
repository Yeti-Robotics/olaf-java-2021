  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinchRollerConstants;

public class PinchRollerSubsystem extends SubsystemBase {
  /** Creates a new PinchRollerSubsytem. */
  private VictorSPX pinchRollerVictor;

public PinchRollerSubsystem() {
    pinchRollerVictor = new VictorSPX(PinchRollerConstants.PINCH_ROLLER_VICTOR);
  }

  public void pinchIn(){
    pinchRollerVictor.set(ControlMode.PercentOutput, PinchRollerConstants.PINCH_ROLLER_IN_SPEED);
  }

  public void pinchOut(){
    pinchRollerVictor.set(ControlMode.PercentOutput, PinchRollerConstants.PINCH_ROLLER_OUT_SPEED);
  }

  public void pinchStop(){
    pinchRollerVictor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}