// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngle extends CommandBase {
  /** Creates a new SetHoodAngle. */
  private HoodSubsystem hoodSubsystem;
  private double angle;
  private double power;
  private double encoderGoal;
  private enum HoodMotion{
    FORWARD, BACKWARD, STILL
  }
  private HoodMotion hoodMotion;

  public SetHoodAngle(HoodSubsystem hoodSubsystem, double angle, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    hoodSubsystem = this.hoodSubsystem;
    angle = this.angle;
    power = Math.abs(this.power);
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderGoal = hoodSubsystem.hoodEncoderFromAngle(angle);
    if (encoderGoal < hoodSubsystem.getHoodEncoder()){
      hoodMotion = HoodMotion.BACKWARD;
      power = -power;
    } else if (encoderGoal > hoodSubsystem.getHoodEncoder()){
      hoodMotion = HoodMotion.FORWARD;
    } else {
      hoodMotion = HoodMotion.STILL;
      power = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hoodSubsystem.moveHood(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hoodMotion == HoodMotion.FORWARD){
      return (encoderGoal >= hoodSubsystem.getHoodEncoder());
    } else if (hoodMotion == HoodMotion.BACKWARD){
      return (encoderGoal <= hoodSubsystem.getHoodEncoder());
    }else {
      return true;
    }
  }
}