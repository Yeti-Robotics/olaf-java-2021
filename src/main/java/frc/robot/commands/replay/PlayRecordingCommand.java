/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.replay;

import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PlayRecordingCommand extends CommandBase {
  /**
   * Creates a new PlayRecordingCommand.
   */
  private String filename;
	private List<RobotInput> localInputSequence;
	private int parsingIndex;
  private boolean[] activatedButtons = new boolean[33];
  private DrivetrainSubsystem drivetrainSubsystem;

  public PlayRecordingCommand(String filename, DrivetrainSubsystem drivetrainSubsystem) {
    this.filename = filename;
    this.drivetrainSubsystem = drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < 33; i++) {
			activatedButtons[i] = false;
		}
		parsingIndex = 0;
		if (!filename.equals(new String(""))) {
			try {
				File file = new File(filename);
				FileInputStream fs = new FileInputStream(file);
				ObjectInputStream os = new ObjectInputStream(fs);
				localInputSequence = (List<RobotInput>) os.readObject();
				os.close();
			} catch (Exception e) {
				e.printStackTrace();
			}
		} else {
			localInputSequence = Robot.recentInputSequence;
		}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (localInputSequence.size() != 0) {
			drivetrainSubsystem.tankDrive(
					-Constants.USER_SPEED_CONTROL_MODIFIER
							* localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[0],
					Constants.USER_SPEED_CONTROL_MODIFIER
							* localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[1]);
			for (int i = 0; i < 3; i++) {
				for (int j = 1; j <= 11; j++) {
					if (parsingIndex != 0 && parsingIndex != 1) {
						for (int a = 0; a < 33; a++) {
							activatedButtons[a] = false;
						}
						if (isButtonPressed(i, j) && !activatedButtons[i * 11 + j]) {
							switch (i) {
								case 0:
									RobotInput.elicitCommand(m_robotContainer.driverStationJoystick, j);
									activatedButtons[i * 11 + j - 1] = true;
									break;
								case 1:
									RobotInput.elicitCommand(Joysticks.RIGHT, j);
									activatedButtons[i * 11 + j - 1] = true;
									break;
								case 2:
									RobotInput.elicitCommand(Joysticks.ARM, j);
									activatedButtons[i * 11 + j - 1] = true;
							}
						}
					}
				}
			}
		}
		parsingIndex++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
		return parsingIndex >= localInputSequence.size() - 1;
  }

  private boolean isButtonPressed(int i, int j) {
		switch (i) {
			case 0:
				if (localInputSequence.get(parsingIndex).getButtonStatus(Joysticks.LEFT, j) == true
						&& localInputSequence.get(parsingIndex - 1).getButtonStatus(Joysticks.LEFT, j) == true
						&& localInputSequence.get(parsingIndex - 2).getButtonStatus(Joysticks.LEFT, j) == true) {
					return true;
				}
			case 1:
				if (localInputSequence.get(parsingIndex).getButtonStatus(Joysticks.RIGHT, j) == true
						&& localInputSequence.get(parsingIndex - 1).getButtonStatus(Joysticks.RIGHT, j) == true
						&& localInputSequence.get(parsingIndex - 2).getButtonStatus(Joysticks.RIGHT, j) == true) {
					return true;
				}
			case 2:
				if (localInputSequence.get(parsingIndex).getButtonStatus(Joysticks.ARM, j) == true
						&& localInputSequence.get(parsingIndex - 1).getButtonStatus(Joysticks.ARM, j) == true
						&& localInputSequence.get(parsingIndex - 2).getButtonStatus(Joysticks.ARM, j) == true) {
					return true;
				}
			default:
				return false;
		}

	}
}
