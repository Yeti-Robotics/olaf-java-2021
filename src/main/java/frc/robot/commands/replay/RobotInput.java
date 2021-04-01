package frc.robot.commands.replay;

import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class RobotInput implements Serializable {
	private static final long serialVersionUID = 1L;

	public static enum Joystick {
		LEFT, RIGHT, ARM
	}

	private static boolean isRecording = false;
	private double leftY, rightY, armY;
	private double leftX, rightX;
	public boolean[] joystickButtons = new boolean[11];
	public static CommandBase[] leftCommands = new CommandBase[11];
	public static CommandBase[] rightCommands = new CommandBase[11];
	public static CommandBase[] armCommands = new CommandBase[11];

	public static boolean getRecordingState() {
		return isRecording;
	}

	public static void setRecordingState(boolean isRecording) {
		RobotInput.isRecording = isRecording;
	}

	public void setButtonState(int button, boolean state) {
		button--;
		joystickButtons[button] = state;
	}

	public void setJoystickXAxis(Joystick driverStationJoystick, double val) {
		switch (driverStationJoystick) {
			case LEFT:
				leftX = val;
				break;
			case RIGHT:
				rightX = val;
				break;
			case ARM:
				break;
		}
	}

	public double[] getJoysticksXAxisStatus() {
		return new double[]{ leftX, rightX};
	}

	public void setJoystickYAxis(Joystick driverStationJoystick, double val) {
		switch (driverStationJoystick) {
			case LEFT:
				leftY = val;
				break;
			case RIGHT:
				rightY = val;
				break;
			case ARM:
				armY = val;
				break;
		}
	}

	public double[] getJoysticksYAxisStatus() {
		return new double[]{ leftY, rightY, armY };
	}

	public boolean getButtonStatus(Joystick joystick, int button) {
		button--;
		return joystickButtons[button];
	}

	public static void elicitCommand(Joystick joystick, int button) {
		button--;
		switch (joystick) {
			case LEFT:
				CommandScheduler.getInstance().schedule(RobotInput.leftCommands[button]);
				break;
			case RIGHT:
				CommandScheduler.getInstance().schedule(RobotInput.rightCommands[button]);
				break;
			case ARM:
				CommandScheduler.getInstance().schedule(RobotInput.armCommands[button]);
				break;
		}
	}
}