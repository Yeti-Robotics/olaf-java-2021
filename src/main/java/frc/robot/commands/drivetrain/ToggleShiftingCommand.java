// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShiftingGearSubsystem;

public class ToggleShiftingCommand extends CommandBase {
  private final ShiftingGearSubsystem shiftingGearSubsystem;

  /** Creates a new ToggleShiftingCommand. */
  public ToggleShiftingCommand(ShiftingGearSubsystem shiftingGearSubsystem) {
    this.shiftingGearSubsystem = shiftingGearSubsystem;
        addRequirements(shiftingGearSubsystem);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH) {
    shiftingGearSubsystem.shiftDown();
} else {
    shiftingGearSubsystem.shiftUp();
}}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
