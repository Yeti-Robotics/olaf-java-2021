package org.usfirst.frc.team3506.robot.commands.domain;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;


public class RobotInput implements Serializable {
	private static final long serialVersionUID = 1L;

	public static enum Joysticks {
		LEFT, RIGHT, ARM
	};

	private double leftY, rightY, armY;
	public boolean[] leftButtons = new boolean[11];
	public boolean[] rightButtons = new boolean[11];
	public boolean[] armButtons = new boolean[11];
	public static edu.wpi.first.wpilibj2.command.Command[] leftCommands = new Command[11];
	public static Command[] rightCommands = new Command[11];
	public static Command[] armCommands = new Command[11];

	public void setButtonState(Joysticks joystick, int button, boolean state) {
		button--;
		switch (joystick) {
			case LEFT:
				leftButtons[button] = state;
				break;
			case RIGHT:
				rightButtons[button] = state;
				break;
			case ARM:
				armButtons[button] = state;
				break;
		}
	}

	public void setJoystickYAxis(Joysticks joystick, double val) {
		switch (joystick) {
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
		double[] joysticksYAxisStatus = { leftY, rightY, armY };
		return joysticksYAxisStatus;
	}

	public boolean getButtonStatus(Joysticks joystick, int button) {
		button--;
		switch (joystick) {
			case LEFT:
				return leftButtons[button];
			case RIGHT:
				return rightButtons[button];
			case ARM:
				return armButtons[button];
			default:
				return false;
		}
	}

	public static void elicitCommand(Joysticks joystick, int button) {
		button--;
		switch (joystick) {
			case LEFT:
				Scheduler.getInstance().add(RobotInput.leftCommands[button]);
				break;
			case RIGHT:
				Scheduler.getInstance().add(RobotInput.rightCommands[button]);
				break;
			case ARM:
				Scheduler.getInstance().add(RobotInput.armCommands[button]);
				break;
		}
	}
}