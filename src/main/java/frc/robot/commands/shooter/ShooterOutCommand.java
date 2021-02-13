// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOutCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  public ShooterOutCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.shoot();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShoot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
