// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetHoodCalculatedAngleCommand extends CommandBase {
  
  private ShooterSubsystem shooterSubsystem;
  private HoodSubsystem hoodSubsystem;

  public SetHoodCalculatedAngleCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(shooterSubsystem, hoodSubsystem);
  }

  @Override
  public void initialize() {
    new SetHoodAngle(hoodSubsystem, shooterSubsystem.calcHoodAngle(), 1.0);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
