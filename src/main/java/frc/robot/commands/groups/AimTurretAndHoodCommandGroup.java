// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hood.SetCalcHoodAngleCommand;
import frc.robot.commands.turret.TurnToTargetPIDCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimTurretAndHoodCommandGroup extends ParallelCommandGroup {
  /** Creates a new AimTurretAndHoodCommand. */
  public AimTurretAndHoodCommandGroup(TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnToTargetPIDCommand(turretSubsystem),
      new SetCalcHoodAngleCommand(hoodSubsystem, 0.1)
    );
  }
}
