// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.StopDriveCommand;
import frc.robot.commands.intake.HopperInCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.shooter.PinchRollerInCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.Limelight;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Joystick driverStationJoystick;
  public DriveSubsystem driveSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public IntakeSubsystem intakeSubsystem;
  public Limelight limelight;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverStationJoystick = new Joystick(OIConstants.DRIVER_STATION_JOY);

    driveSubsystem = new DriveSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    limelight = new Limelight();

    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.tankDrive(getLeftY(), getRightY()), driveSubsystem));
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    setJoystickButtonWhileHeld(driverStationJoystick, 1, new IntakeInCommand(intakeSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 2, new HopperInCommand(intakeSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 3, new PinchRollerInCommand(shooterSubsystem));
  }

  public double getLeftY() {
    if(driverStationJoystick.getRawAxis(1) >= .1 || driverStationJoystick.getRawAxis(1) <= -.1){
      return driverStationJoystick.getRawAxis(1);
    }else{
      return 0;
    }
  }

  public double getLeftX() {
    return driverStationJoystick.getX();
  }

  public double getRightY() {

    if(driverStationJoystick.getRawAxis(3) >= .1 || driverStationJoystick.getRawAxis(3) <= -.1){
      return driverStationJoystick.getRawAxis(3);
    }else{
      return 0;
    }
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
