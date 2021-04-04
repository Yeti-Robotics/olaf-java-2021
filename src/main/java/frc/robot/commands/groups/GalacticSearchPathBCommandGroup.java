// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakePistonCommand;
import frc.robot.commands.replay.PlayRecordingCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchPathBCommandGroup extends SequentialCommandGroup {
  /** Creates a new GalacticSearchCommandGroup. */
  public GalacticSearchPathBCommandGroup(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleIntakePistonCommand(intakeSubsystem),
      new ParallelCommandGroup(
        new IntakeInCommand(intakeSubsystem),
        new PlayRecordingCommand(getRecording(), drivetrainSubsystem))
    );
  }

  @Override
  public void execute() {
      super.execute();
      System.out.println(getRecording());
  }

  public String getRecording(){
    if (Robot.pathColor == Robot.PathColor.RED){
      //path a red
      return "pathBredgalacticsearch.txt";
    } else if (Robot.pathColor == Robot.PathColor.BLUE){
      return "pathBbluegalacticsearch.txt";
    } else{
      return null;
    }
  }
}
