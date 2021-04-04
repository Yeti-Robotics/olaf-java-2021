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
  private String filename;
  private List<RobotInput> localInputSequence;
  private int parsingIndex;
  private boolean[] activatedButtons = new boolean[33];
  public PlayGalacticSearchPathCommand(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = this.drivetrainSubsystem;
    // addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // new PlayRecordingCommand(getRecording(), drivetrainSubsystem).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.pathType == PathType.PATHA && Robot.pathColor == PathColor.RED){
      new PlayRecordingCommand("pathAredgalacticsearch.txt",drivetrainSubsystem);
    } else if (Robot.pathType == PathType.PATHA && Robot.pathColor == PathColor.BLUE){
      new PlayRecordingCommand("pathAbluegalacticsearch.txt", drivetrainSubsystem);
    } else if (Robot.pathType == PathType.PATHB && Robot.pathColor == PathColor.BLUE){
      new PlayRecordingCommand("pathBbluegalacticsearch.txt", drivetrainSubsystem);
    } else{
      new PlayRecordingCommand("pathBredgalacticsearch.txt", drivetrainSubsystem);
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
