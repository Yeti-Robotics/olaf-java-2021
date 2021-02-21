// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PinchRollerSubsystem;

public class AllInCommand extends CommandBase {
  public final PinchRollerSubsystem pinchRollerSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final HopperSubsystem hopperSubsystem;
  public AllInCommand(PinchRollerSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
    this.pinchRollerSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem, hopperSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hopperSubsystem.hopperIn();
    pinchRollerSubsystem.pinchIn();
  }

  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.hopperStop();
    pinchRollerSubsystem.pinchStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
