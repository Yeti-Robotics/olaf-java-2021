// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.galsearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistancePIDCommand;
import frc.robot.commands.drivetrain.TurnForAnglePIDCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathBBlueCommandGroup extends SequentialCommandGroup {
  /** Creates a new PathBBlueCommandGroup. */
  public PathBBlueCommandGroup(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnForAnglePIDCommand(drivetrainSubsystem,-78.69006753),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 152.9705854),
      new IntakeInCommand(intakeSubsystem).withTimeout(.4),
      new TurnForAnglePIDCommand( drivetrainSubsystem, 123.69006753),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 84.85281374),
      new IntakeInCommand(intakeSubsystem).withTimeout(.4),
      new TurnForAnglePIDCommand(drivetrainSubsystem, -90),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 84.85281374),
      new IntakeInCommand(intakeSubsystem).withTimeout(.4),
      new TurnForAnglePIDCommand(drivetrainSubsystem, 45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 30),
      new IntakeInCommand(intakeSubsystem).withTimeout(.4)
    );
  }
}
