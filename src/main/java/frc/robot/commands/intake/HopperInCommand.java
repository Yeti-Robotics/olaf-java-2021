// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class HopperInCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  public HopperInCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.hopperIn();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.hopperStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
