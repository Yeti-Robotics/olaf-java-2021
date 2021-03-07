// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pinchroller;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PinchRollerSubsystem;

public class PinchRollerInCommand extends CommandBase {
  private final PinchRollerSubsystem pinchRollerSubsystem;
  public PinchRollerInCommand(PinchRollerSubsystem shooterSubsystem) {
    this.pinchRollerSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pinchRollerSubsystem.pinchIn();
  }

  @Override
  public void end(boolean interrupted) {
    pinchRollerSubsystem.pinchStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
