// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.utils.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetCalcHoodAnglePIDCommand extends PIDCommand {
  /** Creates a new SetCalcHoodAnglePIDCommand. */
  private HoodSubsystem hoodSubsystem;
  public SetCalcHoodAnglePIDCommand(HoodSubsystem hoodSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(.005, 0, 0),
        // This should return the measurement
        hoodSubsystem::getEncoder,
        // This should return the setpoint (can also be a constant)
        hoodSubsystem.hoodEncoderFromAngle(hoodSubsystem.calcHoodAngle(Limelight.getHorDistance())),
        // This uses the output
        output -> {
          hoodSubsystem.moveHood(output);
          System.out.println("Output: "+ output + "; setpoint: " + hoodSubsystem.hoodEncoderFromAngle(hoodSubsystem.calcHoodAngle(Limelight.getHorDistance())) + "; Current angle: " + hoodSubsystem.hoodAngleFromEncoder(hoodSubsystem.getEncoder()));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    this.hoodSubsystem = hoodSubsystem;
    getController().setTolerance(HoodConstants.HOOD_ANGLE_TOLERANCE);
    addRequirements(hoodSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
