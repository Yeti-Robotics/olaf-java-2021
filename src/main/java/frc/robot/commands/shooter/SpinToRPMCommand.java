// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpinToRPMCommand extends PIDCommand {
  private final ShooterSubsystem shooterSubsystem;
  
  public SpinToRPMCommand(ShooterSubsystem shooterSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.0, 0.0, 0.0),
        // This should return the measurement
        shooterSubsystem::getFlywheelRPM,
        // This should return the setpoint (can also be a constant)
        ShooterConstants.MAX_RPM,
        // This uses the output
        output -> {
          // Use the output here
          System.out.println(output);
          shooterSubsystem.shootFlywheel(0.85 + output);
        });
    this.shooterSubsystem = shooterSubsystem;
    //getController().setTolerance(10.0);
    addRequirements(shooterSubsystem); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
