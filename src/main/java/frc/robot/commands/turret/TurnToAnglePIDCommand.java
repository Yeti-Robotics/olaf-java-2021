// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAnglePIDCommand extends PIDCommand {
  private TurretSubsystem turretSubsystem;

  /** Creates a new TurnToAnglePIDCommand. */
  public TurnToAnglePIDCommand(TurretSubsystem turretSubsystem, double angle) {
    super(
        // Tune values later
        new PIDController(0, 0, 0),
        // This should return the measurement
        turretSubsystem::getEncoder,
        // This should return the setpoint (can also be a constant)
        turretSubsystem.turretEncoderFromAngle(angle),
        // This uses the output
        output -> {
          if(angle > 0){
            turretSubsystem.moveTurret(output);
          } else {
            turretSubsystem.moveTurret(-output);
          }
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.turretSubsystem = turretSubsystem;
    getController().setTolerance(1.0);
  }

  @Override
  public void initialize() {
    super.initialize();
    turretSubsystem.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
