// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.galsearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnForAngleCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakePistonCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathBRedCommandGroup extends SequentialCommandGroup {
  /** Creates a new PathBRedCommandGroup. */
  public PathBRedCommandGroup(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleIntakePistonCommand(intakeSubsystem),
      new DriveForDistanceCommand(drivetrainSubsystem, 60, 0.7),
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      new TurnForAngleCommand(drivetrainSubsystem,56.31, .5,.5 ),
      new DriveForDistanceCommand(drivetrainSubsystem, 108.166, 0.7),
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      new TurnForAngleCommand(drivetrainSubsystem,-112.62,  .5,.5 ),
      new DriveForDistanceCommand(drivetrainSubsystem, 108.166, 0.7),
      new IntakeInCommand(intakeSubsystem).withTimeout(0.4),
      new TurnForAngleCommand(drivetrainSubsystem,56.31,  .5,.5 ),
      new DriveForDistanceCommand(drivetrainSubsystem, 120, 1)
    );
  }
}
