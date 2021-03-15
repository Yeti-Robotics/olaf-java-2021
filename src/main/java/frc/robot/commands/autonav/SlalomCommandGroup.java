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
public class SlalomCommandGroup extends SequentialCommandGroup {
  /** Creates a new SlalomCommandGroup. */
  public SlalomCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForDistanceCommand(drivetrainSubsystem, 50, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem, 45,.5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 51.96152 , 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,-45, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 120, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,-45, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 84.852813 , 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,78.690067, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 54.0833, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,202.6198, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 54.0833, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,45, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 84.852813, .5),
      new TurnForAngleCommand( drivetrainSubsystem,45,.5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 120, .5),
      new TurnForAngleCommand(drivetrainSubsystem,-45, .5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 51.9615242, .5),
      new TurnForAngleCommand(drivetrainSubsystem,45,.5,.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 60, .5)
    );
  }
}
