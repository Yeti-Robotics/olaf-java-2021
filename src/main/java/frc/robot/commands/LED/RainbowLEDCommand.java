// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
public class RainbowLEDCommand extends CommandBase {
  /** Creates a new RainbowCommand. */
  private LEDSubsystem ledSubsystem;
  private int rainbowFirstPixelHue;
  public RainbowLEDCommand(LEDSubsystem ledSubsystem, int rainbowFirstPixelHue) {
    this.ledSubsystem = ledSubsystem;
    this.rainbowFirstPixelHue = rainbowFirstPixelHue;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // For every pixel
      for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final int hue = (rainbowFirstPixelHue + (i * 180 / ledSubsystem.getBufferLength())) % 180;
        // Set the value
        ledSubsystem.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      rainbowFirstPixelHue += 3;
      // Check bounds
      rainbowFirstPixelHue %= 180;
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
