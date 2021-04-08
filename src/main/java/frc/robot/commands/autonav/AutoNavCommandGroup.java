// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonav;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ToggleIntakePistonCommand;
import frc.robot.commands.replay.PlayRecordingCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoNavCommandGroup extends SequentialCommandGroup {

  private DrivetrainSubsystem drivetrainSubsystem;
  /** Creates a new AutoNavCommandGroup. */
  public AutoNavCommandGroup(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrainSubsystem = drivetrainSubsystem;
    addCommands(
      new ToggleIntakePistonCommand(intakeSubsystem),
      new PlayRecordingCommand("speedybounce.txt", drivetrainSubsystem)
    );
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    drivetrainSubsystem.stopDrive();
  }
}
