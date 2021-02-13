// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AllInCommand extends CommandBase {
  public final ShooterSubsystem shooterSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public AllInCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.hopperIn();
    shooterSubsystem.pinchIn();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.hopperStop();
    shooterSubsystem.pinchStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
