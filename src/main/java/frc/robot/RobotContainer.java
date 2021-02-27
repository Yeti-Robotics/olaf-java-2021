// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AllInCommand;
import frc.robot.commands.hood.TestHoodCommand;
import frc.robot.commands.hopper.HopperInCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakePistonCommand;
import frc.robot.commands.shooter.PinchRollerInCommand;
import frc.robot.commands.shooter.TestShootingCommand;
import frc.robot.commands.shooter.ToggleShooterOnOffCommand;
import frc.robot.commands.turret.TurretTestCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Joystick driverStationJoystick;
  public DrivetrainSubsystem drivetrainSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public IntakeSubsystem intakeSubsystem;
  public HopperSubsystem hopperSubsystem;
  public Limelight limelight;
  public PinchRollerSubsystem pinchRollerSubsystem;
  public HoodSubsystem hoodSubsystem;
  public TurretSubsystem turretSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driverStationJoystick = new Joystick(OIConstants.DRIVER_STATION_JOY);

    shooterSubsystem = new ShooterSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    drivetrainSubsystem = new DrivetrainSubsystem();
    hopperSubsystem = new HopperSubsystem();
    limelight = new Limelight();
    pinchRollerSubsystem = new PinchRollerSubsystem();
    hoodSubsystem = new HoodSubsystem();
    turretSubsystem = new TurretSubsystem();

    drivetrainSubsystem
        .setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()), drivetrainSubsystem));
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    setJoystickButtonWhileHeld(driverStationJoystick, 1, new IntakeInCommand(intakeSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 2, new HopperInCommand(hopperSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 3, new PinchRollerInCommand(pinchRollerSubsystem));
    setJoystickButtonWhenPressed(driverStationJoystick, 4, new ToggleShooterOnOffCommand(shooterSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 5, new AllInCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
    setJoystickButtonWhenPressed(driverStationJoystick, 6, new ToggleIntakePistonCommand(intakeSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 7, new TurretTestCommand(turretSubsystem, .3));
    setJoystickButtonWhileHeld(driverStationJoystick, 8, new TestHoodCommand(hoodSubsystem, .3));
    setJoystickButtonWhileHeld(driverStationJoystick, 9, new TestHoodCommand(hoodSubsystem, -.3));

  }

  public double getLeftY() {
    if (driverStationJoystick.getRawAxis(1) >= .1 || driverStationJoystick.getRawAxis(1) <= -.1) {
      return driverStationJoystick.getRawAxis(1);
    } else {
      return 0;
    }
  }

  public double getLeftX() {
    return driverStationJoystick.getX();
  }

  public double getRightY() {
      return driverStationJoystick.getRawAxis(3);
  }

  public double getRightX() {
    return driverStationJoystick.getX();
  }

  private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenPressed(command);
  }

  private void setJoystickButtonWhileHeld(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whileHeld(command);
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
