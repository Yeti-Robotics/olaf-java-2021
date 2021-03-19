package frc.robot.commands.replay;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class RobotInput implements Serializable {
	private static final long serialVersionUID = 1L;

	public static enum Joysticks {
		LEFT, RIGHT, ARM
	};

	private double leftY, rightY, armY;
	public boolean[] leftButtons = new boolean[11];
	public boolean[] rightButtons = new boolean[11];
	public boolean[] armButtons = new boolean[11];
	public static CommandBase[] leftCommands = new CommandBase[11];
	public static CommandBase[] rightCommands = new CommandBase[11];
	public static CommandBase[] armCommands = new CommandBase[11];

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