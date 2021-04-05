// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.galsearch;

import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Robot.PathColor;
import frc.robot.Robot.PathType;
import frc.robot.commands.replay.PlayRecordingCommand;
import frc.robot.commands.replay.RobotInput;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PlayGalacticSearchPathCommand extends CommandBase {
  /** Creates a new PlayGalacticSearchPathCommand. */
  private DrivetrainSubsystem drivetrainSubsystem;
  private IntakeSubsystem intakeSubsystem;
  public PlayGalacticSearchPathCommand(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.intakeSubsystem= intakeSubsystem;
    addRequirements(this.intakeSubsystem, this.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // new PlayRecordingCommand(getRecording(), drivetrainSubsystem).schedule();
      intakeSubsystem.intakeIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.pathType == PathType.PATHA && Robot.pathColor == PathColor.RED){
      new PlayRecordingCommand("oldredA.txt",drivetrainSubsystem).schedule();
    } else if (Robot.pathType == PathType.PATHA && Robot.pathColor == PathColor.BLUE){
      new PlayRecordingCommand("oldblueA.txt", drivetrainSubsystem).schedule();
    } else if (Robot.pathType == PathType.PATHB && Robot.pathColor == PathColor.BLUE){
      new PlayRecordingCommand("oldblueB.txt", drivetrainSubsystem).schedule();
    } else{
      new PlayRecordingCommand("oldredB.txt", drivetrainSubsystem).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  private String getRecording(){
    if (Robot.pathType == PathType.PATHA && Robot.pathColor == PathColor.RED){
      return "pathAredgalacticsearch.txt";
    } else if (Robot.pathType == PathType.PATHA && Robot.pathColor == PathColor.BLUE){
      return "pathAbluegalacticsearch.txt";
    } else if (Robot.pathType == PathType.PATHB && Robot.pathColor == PathColor.BLUE){
      return "pathBbluegalacticsearch.txt";
    } else{
      return "pathBredgalacticsearch.txt";
    }
  }
}
