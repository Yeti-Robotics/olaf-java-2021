// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonav;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistancePIDCommand;
import frc.robot.commands.drivetrain.TurnForAnglePIDCommand;
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
      new DriveForDistancePIDCommand(drivetrainSubsystem, 30),
      new TurnForAnglePIDCommand(drivetrainSubsystem, -26.57),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 30),
      new TurnForAnglePIDCommand(drivetrainSubsystem, 26.57),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 60),
      new TurnForAnglePIDCommand(drivetrainSubsystem,63.43),
      new DriveForDistancePIDCommand(drivetrainSubsystem,60),
      new TurnForAnglePIDCommand( drivetrainSubsystem,-63.43),
      new DriveForDistancePIDCommand(drivetrainSubsystem,30),
      new TurnForAnglePIDCommand( drivetrainSubsystem, -14.04), 
      new DriveForDistancePIDCommand(drivetrainSubsystem,30),
      new TurnForAnglePIDCommand(drivetrainSubsystem,14.04),
      new DriveForDistancePIDCommand(drivetrainSubsystem,30),
      new TurnForAnglePIDCommand( drivetrainSubsystem,75.96),
      new DriveForDistancePIDCommand(drivetrainSubsystem,30),
      new TurnForAnglePIDCommand(drivetrainSubsystem,-76.96),
      new DriveForDistancePIDCommand(drivetrainSubsystem,30),
      new TurnForAnglePIDCommand(drivetrainSubsystem, -14.04),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 30)
    );
  }
}
