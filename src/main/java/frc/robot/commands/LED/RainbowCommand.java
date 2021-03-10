// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;;
public class RainbowCommand extends CommandBase {
  /** Creates a new RainbowCommand. */
  public RainbowCommand() {
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      AddressableLEDBuffer ledBuffer;
      AddressableLED ledStrip;
    // For every pixel
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
        // Set the value
        ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      rainbowFirstPixelHue += 3;
      // Check bounds
      rainbowFirstPixelHue %= 180;
      ledStrip.setData(ledBuffer);
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
