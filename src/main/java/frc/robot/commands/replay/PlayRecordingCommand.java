/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.replay;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.List;

public class PlayRecordingCommand extends CommandBase {
    /**
     * Creates a new PlayRecordingCommand.
     */
    private String filename;
    private List<RobotInput> localInputSequence;
    private int parsingIndex;
    private boolean[] activatedButtons = new boolean[33];
    private DrivetrainSubsystem drivetrainSubsystem;
    private RobotContainer robotContainer;

    public PlayRecordingCommand(String filename, DrivetrainSubsystem drivetrainSubsystem) {
        this.filename = "/home/lvuser/recordings/" + filename;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    public PlayRecordingCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.filename = null;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        for (int i = 0; i < 33; i++) {
            activatedButtons[i] = false;
        }
        parsingIndex = 0;
        if (filename != null) {
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
			switch (drivetrainSubsystem.getDriveMode()) {
				case TANK:
					drivetrainSubsystem.tankDrive(localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[0], localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[1]);
					break;
				case CHEEZY:
					drivetrainSubsystem.cheezyDrive(localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[0], localInputSequence.get(parsingIndex).getJoysticksXAxisStatus()[1]);
					break;
				case ARCADE:
					drivetrainSubsystem.arcadeDrive(localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[0], localInputSequence.get(parsingIndex).getJoysticksXAxisStatus()[1]);
			}
//            drivetrainSubsystem.tankDrive(
//                    -Constants.USER_SPEED_CONTROL_MODIFIER
//                            * localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[0],
//                    Constants.USER_SPEED_CONTROL_MODIFIER
//                            * localInputSequence.get(parsingIndex).getJoysticksYAxisStatus()[0]);

            // for (int i = 0; i < 3; i++) {
            // 	for (int j = 1; j <= 11; j++) {
            // 		if (parsingIndex != 0 && parsingIndex != 1) {
            // 			for (int a = 0; a < 33; a++) {
            // 				activatedButtons[a] = false;
            // 			}
            // 			if (isButtonPressed(i, j) && !activatedButtons[i * 11 + j]) {
            // 				switch (i) {
            // 					case 0:
            // 						RobotInput.elicitCommand(RobotInput.Joystick.LEFT, j);
            // 						activatedButtons[i * 11 + j - 1] = true;
            // 						break;
            // 					case 1:
            // 						RobotInput.elicitCommand(RobotInput.Joystick.RIGHT, j);
            // 						activatedButtons[i * 11 + j - 1] = true;
            // 						break;
            // 					case 2:
            // 						RobotInput.elicitCommand(RobotInput.Joystick.ARM, j);
            // 						activatedButtons[i * 11 + j - 1] = true;
            // 				}
            // 			}
            // 		}
            // 	}
            // }
        }
        parsingIndex++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return parsingIndex >= localInputSequence.size() - 1;
    }

    private boolean isButtonPressed(int i, int j) {
        switch (i) {
            case 0:
                if (localInputSequence.get(parsingIndex).getButtonStatus(RobotInput.Joystick.LEFT, j) == true
                        && localInputSequence.get(parsingIndex - 1).getButtonStatus(RobotInput.Joystick.LEFT, j) == true
                        && localInputSequence.get(parsingIndex - 2).getButtonStatus(RobotInput.Joystick.LEFT, j) == true) {
                    return true;
                }
            case 1:
                if (localInputSequence.get(parsingIndex).getButtonStatus(RobotInput.Joystick.RIGHT, j) == true
                        && localInputSequence.get(parsingIndex - 1).getButtonStatus(RobotInput.Joystick.RIGHT, j) == true
                        && localInputSequence.get(parsingIndex - 2).getButtonStatus(RobotInput.Joystick.RIGHT, j) == true) {
                    return true;
                }
            case 2:
                if (localInputSequence.get(parsingIndex).getButtonStatus(RobotInput.Joystick.ARM, j) == true
                        && localInputSequence.get(parsingIndex - 1).getButtonStatus(RobotInput.Joystick.ARM, j) == true
                        && localInputSequence.get(parsingIndex - 2).getButtonStatus(RobotInput.Joystick.ARM, j) == true) {
                    return true;
                }
            default:
                return false;
        }

    }
}
