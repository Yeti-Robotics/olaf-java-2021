// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AllInCommand extends CommandBase {
  public final ShooterSubsystem shooterSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final HopperSubsystem hopperSubsystem;
  public AllInCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem, hopperSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hopperSubsystem.hopperIn();
    shooterSubsystem.pinchIn();
  }

  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.hopperStop();
    shooterSubsystem.pinchStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
