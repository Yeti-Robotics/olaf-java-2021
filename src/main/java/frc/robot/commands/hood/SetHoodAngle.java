// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngle extends CommandBase {
  /** Creates a new SetHoodAngle. */
  private HoodSubsystem hoodSubsystem;
  private double angle;
  private double power;
  private double encoderGoal;

  public SetHoodAngle(HoodSubsystem hoodSubsystem, double angle, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hoodSubsystem = hoodSubsystem;
    this.angle = angle;
    this.power = Math.abs(power);
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderGoal = hoodSubsystem.hoodEncoderFromAngle(angle);
    if (encoderGoal < hoodSubsystem.getEncoder()){
        power = -power;
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
  return Math.abs(encoderGoal - hoodSubsystem.getEncoder()) <= Constants.HoodConstants.HOOD_ANGLE_TOLERANCE;
  }
}
