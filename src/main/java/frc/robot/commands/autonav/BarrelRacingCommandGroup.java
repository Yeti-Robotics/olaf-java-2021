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
public class BarrelRacingCommandGroup extends SequentialCommandGroup {
  /** Creates a new BarrelRacingCommandGroup. */
  public BarrelRacingCommandGroup(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForDistanceCommand(drivetrainSubsystem, 84, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem, -123.69, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 16.22496 , 0.5),
      new TurnForAngleCommand(drivetrainSubsystem, -112.62, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 16.22496, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,-108.430, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 68.41056 , 0.5),
      new TurnForAngleCommand(drivetrainSubsystem,-108.430, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 21.63336, 0.5),
      new TurnForAngleCommand( drivetrainSubsystem, 112.62, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 21.63336,  0.5),
      new TurnForAngleCommand( drivetrainSubsystem, 70, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 59.397,.5),
      new TurnForAngleCommand( drivetrainSubsystem, 135, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 24,  .5 ),
      new TurnForAngleCommand( drivetrainSubsystem, 90, .5, .5),
      new DriveForDistanceCommand(drivetrainSubsystem, 126, .5)
    );
  }
}
