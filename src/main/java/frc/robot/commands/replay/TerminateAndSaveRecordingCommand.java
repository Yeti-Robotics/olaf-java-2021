/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.replay;

import java.io.File;
import java.io.FileOutputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TerminateAndSaveRecordingCommand extends CommandBase {
  /**
   * Creates a new TerminateAndSaveRecordingCommand.
   */
  public TerminateAndSaveRecordingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.recording = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try{
      File file = new File("/home/lvuser/recordings/" + new Long(System.currentTimeMillis()).toString() + "recording.txt");
      FileOutputStream fs = new FileOutputStream(file);
      ObjectOutputStream os = new ObjectOutputStream(fs);
      os.writeObject(Robot.inputSequence);
      os.close();
    } catch(Exception e){
      e.printStackTrace();
    }
    Robot.recentInputSequence = new ArrayList<RobotInput>();
    for(RobotInput input : Robot.inputSequence){
      Robot.recentInputSequence.add(input);
    }
    Robot.inputSequence.clear();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
