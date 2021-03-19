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
public class BarrelRacingCommandGroup extends SequentialCommandGroup {
  /** Creates a new BarrelRacingCommandGroup. */
  public BarrelRacingCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForDistancePIDCommand(drivetrainSubsystem, 84),
      new TurnForAnglePIDCommand(drivetrainSubsystem, -123.69),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 16.22496),
      new TurnForAnglePIDCommand(drivetrainSubsystem, -112.62),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 16.22496),
      new TurnForAnglePIDCommand(drivetrainSubsystem,-108.430),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 68.41056),
      new TurnForAnglePIDCommand(drivetrainSubsystem,-108.430),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 21.63336),
      new TurnForAnglePIDCommand( drivetrainSubsystem, 112.62),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 21.63336),
      new TurnForAnglePIDCommand( drivetrainSubsystem, 70),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 59.397),
      new TurnForAnglePIDCommand( drivetrainSubsystem, 135),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 24),
      new TurnForAnglePIDCommand( drivetrainSubsystem, 90),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 126)
    );
  }
}
