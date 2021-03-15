// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonav;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnForAngleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BouncePathCommandGroup extends SequentialCommandGroup {
  /** Creates a new BouncePathCommandGroup. */
  public BouncePathCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForDistanceCommand(drivetrainSubsystem, 30, 0.),
      new TurnForAngleCommand(drivetrainSubsystem, -26.57, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 30, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem, 26.57, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 60, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,63.43,  .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem,60, 0.5),
      new TurnForAngleCommand( drivetrainSubsystem,-63.43, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem,30, 0.5),
      new TurnForAngleCommand( drivetrainSubsystem, -14.04,.5,.5), 
      new DriveForDistanceCommand(drivetrainSubsystem,30, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,14.04,  .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem,30, .5),
      new TurnForAngleCommand( drivetrainSubsystem,75.96, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem,30, .5),
      new TurnForAngleCommand(drivetrainSubsystem,-76.96,  .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem,30, .5),
      new TurnForAngleCommand(drivetrainSubsystem, -14.04, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 30, .5)
    );
  }
}
