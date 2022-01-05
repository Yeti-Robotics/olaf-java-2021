// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PinchRollerSubsystem;

public class StopFullIntakeCommand extends CommandBase {

  private IntakeSubsystem intakeSubsystem;
  private HopperSubsystem hopperSubsystem;
  private PinchRollerSubsystem pinchRollerSubsystem;

  public StopFullIntakeCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem, PinchRollerSubsystem pinchRollerSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.pinchRollerSubsystem = pinchRollerSubsystem;
    addRequirements(intakeSubsystem, hopperSubsystem, pinchRollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.intakeStop();
    hopperSubsystem.hopperStop();
    pinchRollerSubsystem.pinchStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return true;
  }
}
