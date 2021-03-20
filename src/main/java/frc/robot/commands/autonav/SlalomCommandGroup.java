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
public class SlalomCommandGroup extends SequentialCommandGroup {
  /** Creates a new SlalomCommandGroup. */
  public SlalomCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForDistancePIDCommand(drivetrainSubsystem, 50),
      new TurnForAnglePIDCommand(drivetrainSubsystem, 45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 51.96152),
      new TurnForAnglePIDCommand(drivetrainSubsystem,-45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 120),
      new TurnForAnglePIDCommand(drivetrainSubsystem,-45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 84.852813),
      new TurnForAnglePIDCommand(drivetrainSubsystem,78.690067),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 54.0833),
      new TurnForAnglePIDCommand(drivetrainSubsystem,202.6198),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 54.0833),
      new TurnForAnglePIDCommand(drivetrainSubsystem,45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 84.852813),
      new TurnForAnglePIDCommand( drivetrainSubsystem,45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 120),
      new TurnForAnglePIDCommand(drivetrainSubsystem,-45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 51.9615242),
      new TurnForAnglePIDCommand(drivetrainSubsystem,45),
      new DriveForDistancePIDCommand(drivetrainSubsystem, 60)
    );
  }
}
