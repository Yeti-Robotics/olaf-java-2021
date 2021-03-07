// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class CalibrateTurretCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;

  public CalibrateTurretCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.moveTurret(0.5);
  }

  @Override
  public void execute() {
    if(turretSubsystem.getPhysicalLimit()){
      turretSubsystem.moveTurret(-0.5);
    }
    if(turretSubsystem.getReverseLimit()){
      turretSubsystem.resetEncoder();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.getReverseLimit();
  }
}
