// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretAngle extends CommandBase {
  /** Creates a new TurretTurnForAngle. */
  private TurretSubsystem turretSubsystem;
  private double angle;
  private double power;
  private double encoderGoal;
  private enum TurretMotion{
    FORWARD, BACKWARD, STILL
  }
  private TurretMotion turretMotion;
  public SetTurretAngle(TurretSubsystem turretSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    turretSubsystem = this.turretSubsystem;
    angle = this.angle;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderGoal = turretSubsystem.turretEncoderFromAngle(angle);
    if (encoderGoal < turretSubsystem.getEncoder()){
      turretMotion = TurretMotion.BACKWARD;
      power = -power;
    } else if (encoderGoal > turretSubsystem.getEncoder()){
      turretMotion = TurretMotion.FORWARD;
    } else {
      turretMotion = TurretMotion.STILL;
      power = 0;
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
    if (turretMotion == TurretMotion.FORWARD){
      return (encoderGoal >= turretSubsystem.getEncoder());
    } else if (turretMotion == TurretMotion.BACKWARD){
      return (encoderGoal <= turretSubsystem.getEncoder());
    }else {
      return true;
    }
  }
}
