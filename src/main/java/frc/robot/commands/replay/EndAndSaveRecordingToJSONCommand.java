// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.replay;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class EndAndSaveRecordingToJSONCommand extends CommandBase {
  /** Creates a new EndAndSaveRecordingToJSONCommand. */
  public EndAndSaveRecordingToJSONCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotInput.setRecordingState(false);
    Robot.recentInputSequence = new ArrayList<RobotInput>();
    Robot.recentInputSequence.addAll(Robot.inputSequence);
    Robot.inputSequence.clear();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String s = System.getProperty("user.dir");
    File file = new File(s +"src/main/java/frc/robot/commands/replay/recordings/" + System.currentTimeMillis() +"recording.json");
    try {
        file.createNewFile();
        System.out.println("s created");
    } catch (Exception e) {
     e.printStackTrace();  
    }
    Path filePath = file.toPath();
    try
    {
        //Write content to file
        Files.writeString(filePath, "{ "+'"'+ "path" + '"' + ": " + Robot.recentInputSequence.toString() + "}", StandardOpenOption.APPEND);
        //Verify file content
        String content = Files.readString(filePath);
        System.out.println(content);
    } 
    catch (Exception e) 
    {
        e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
