// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class GradientLEDCommand extends CommandBase {
  /** Creates a new GradientLEDCommand. */
  private LEDSubsystem ledSubsystem;
  private int firstHueValue;
  private int lastHueValue;
  public GradientLEDCommand(LEDSubsystem ledSubsystem, int firstHueValue, int lastHueValue) {
    this.ledSubsystem = ledSubsystem;
    this.firstHueValue = firstHueValue;
    this.lastHueValue = lastHueValue;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (firstHueValue + (i * 180 / ledSubsystem.getBufferLength())) % 180;
      // Set the value
      ledSubsystem.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    firstHueValue += 3;
    // Check bounds
    firstHueValue %= 180;
    ledSubsystem.sendData();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
