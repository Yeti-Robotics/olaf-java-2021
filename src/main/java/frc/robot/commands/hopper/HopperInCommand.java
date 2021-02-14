// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperInCommand extends CommandBase {
  private final HopperSubsystem hopperSubsystem;
  public HopperInCommand(HopperSubsystem hopperSubsystem) {
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(hopperSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hopperSubsystem.hopperIn();
  }

  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.hopperStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
