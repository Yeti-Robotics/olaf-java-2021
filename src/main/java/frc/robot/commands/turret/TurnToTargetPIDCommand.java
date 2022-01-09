// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTargetPIDCommand extends PIDCommand {
  private TurretSubsystem turretSubsystem;

  /** Creates a new TurnToTargetPIDTurret. */
  public TurnToTargetPIDCommand(TurretSubsystem turretSubsystem){
    super(
        // Tune values later
        new PIDController(TurretConstants.kPTurretVel, TurretConstants.kITurretVel, TurretConstants.kDTurretVel),
        // This should return the measurement
        Limelight::getTx,
        // This should return the setpoint (can also be a constant)
        0.0,
        // This uses the output
        output -> {
          turretSubsystem.moveTurret(-output);
        }
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.turretSubsystem = turretSubsystem;
    getController().enableContinuousInput(-28,28);
    getController().setTolerance(.25);
    addRequirements(turretSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("i am aligned (supposedly)");
    return getController().atSetpoint() || turretSubsystem.getPhysicalLimit();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
  }
}
