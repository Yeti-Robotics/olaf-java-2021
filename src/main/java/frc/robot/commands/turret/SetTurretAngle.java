// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretAngle extends CommandBase {
  /** Creates a new TurretTurnForAngle. */
  private TurretSubsystem turretSubsystem;
  private double angle;
  private double power;
  private double encoderGoal;
  public SetTurretAngle(TurretSubsystem turretSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.angle = angle;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderGoal = turretSubsystem.turretEncoderFromAngle(angle);
    if (encoderGoal < turretSubsystem.getEncoder()){
      power = -power;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.moveTurret(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(encoderGoal - turretSubsystem.getEncoder()) <= Constants.TurretConstants.TURRET_ANGLE_THRESHOLD;
  }
}
