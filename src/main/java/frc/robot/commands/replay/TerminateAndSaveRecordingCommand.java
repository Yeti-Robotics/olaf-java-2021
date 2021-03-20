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
    RobotInput.setRecordingState(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      String filename = "/home/lvuser/recordings/" + System.currentTimeMillis() + "recording.txt";
      File file = new File(filename);
      FileOutputStream fs = new FileOutputStream(file);
      ObjectOutputStream os = new ObjectOutputStream(fs);
      os.writeObject(Robot.inputSequence);
      os.close();
      fs.close();
      System.out.println("i got a file baby");

      file = new File(filename + ".csv");
      fs = new FileOutputStream(file);
      StringBuilder out = new StringBuilder();
      out.append("index, leftX, leftY, rightX, rightY");
      for (int i = 0; i < Robot.inputSequence.size(); i++) {
        double[] joystickX = Robot.inputSequence.get(i).getJoysticksXAxisStatus();
        double[] joystickY = Robot.inputSequence.get(i).getJoysticksYAxisStatus();
        out.append(String.format("%d, %f, %f, %f, %f\n", i, joystickX[0], joystickY[0], joystickX[1], joystickY[1]));
      }
      fs.write(out.toString().getBytes());
      fs.close();
    } catch(Exception e) {
      e.printStackTrace();
      System.out.println("im fucking broken");
    }
    Robot.recentInputSequence = new ArrayList<RobotInput>();
    Robot.recentInputSequence.addAll(Robot.inputSequence);
    Robot.inputSequence.clear();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("i've supposedly finished saved my file");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
