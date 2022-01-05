// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AllInCommand;
import frc.robot.commands.shooter.ShootingCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PinchRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireBallCommandGroup extends SequentialCommandGroup {
  public FireBallCommandGroup(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem, PinchRollerSubsystem pinchRollerSubsystem) {
    addCommands(
      new ShootingCommand(shooterSubsystem), 
      new AllInCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem)
    );
  }
}
