// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooterCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  public ToggleShooterCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (ShooterSubsystem.getShooterStatus() == ShooterSubsystem.shooterStatus.OFF) {
      shooterSubsystem.shootFlywheel();
    } else {
      shooterSubsystem.stopFlywheel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override 
  public boolean isFinished() {
    return true;
  }
}
