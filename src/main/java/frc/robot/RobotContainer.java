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
import frc.robot.commands.AllOutCommand;
import frc.robot.commands.autonav.BarrelRacingCommandGroup;
import frc.robot.commands.drivetrain.ToggleShiftingCommand;
import frc.robot.commands.groups.FireBallCommandGroup;
import frc.robot.commands.hood.TestHoodCommand;
import frc.robot.commands.hopper.HopperInCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakePistonCommand;
import frc.robot.commands.pinchroller.PinchRollerInCommand;
import frc.robot.commands.shooter.SpinToRPMCommand;
import frc.robot.commands.shooter.StopFullIntakeCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.commands.shooter.ShootingCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.commands.turret.CalibrateTurretCommand;
import frc.robot.commands.turret.TurnToTargetPIDCommand;
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
  public LEDSubsystem ledSubsystem;
  public ShiftingGearSubsystem shiftingGearSubsystem;

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
    ledSubsystem = new LEDSubsystem();
    shiftingGearSubsystem = new ShiftingGearSubsystem();
    
    drivetrainSubsystem
        .setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.cheezyDrive(getLeftY(), getRightX()), drivetrainSubsystem));
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    //joystick buttons (primary driver)
    setJoystickButtonWhenPressed(driverStationJoystick, 11, new ToggleShiftingCommand(shiftingGearSubsystem));
    // setJoystickButtonWhenPressed(driverStationJoystick, 12, ); //toggle intake piston down, intake and slow hopper run

    //secondary buttons
    // setJoystickButtonWhenPressed(driverStationJoystick, 1, new BarrelRacingCommandGroup(drivetrainSubsystem));
    setJoystickButtonWhenPressed(driverStationJoystick, 1, new FireBallCommandGroup(shooterSubsystem, intakeSubsystem, hopperSubsystem, pinchRollerSubsystem)); //intakeSubsystem, hopperSubsystem, pinchRollerSubsystem));
    // setJoystickButtonWhileHeld(driverStationJoystick, 2, new AllOutCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem)); //reverse everything
    setJoystickButtonWhileHeld(driverStationJoystick, 2, new AllOutCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 3, new StopShooterCommand(shooterSubsystem));
    setJoystickButtonWhenPressed(driverStationJoystick, 6, new StopFullIntakeCommand(intakeSubsystem, hopperSubsystem, pinchRollerSubsystem)); 
    setJoystickButtonWhileHeld(driverStationJoystick, 4, new TestHoodCommand(hoodSubsystem, 0.05)); //hood out
    setJoystickButtonWhileHeld(driverStationJoystick, 5, new TestHoodCommand(hoodSubsystem, -0.05)); //hood in
    // setJoystickButtonWhileHeld(driverStationJoystick, 6, new AllInCommand(pinchRollerSubsystem, intakeSubsystem, hopperSubsystem)); //shoot
    setJoystickButtonWhenPressed(driverStationJoystick, 7, new TurnToTargetPIDCommand(turretSubsystem)); //aim
    setJoystickButtonWhenPressed(driverStationJoystick, 8, new ToggleIntakePistonCommand(intakeSubsystem)); //temp
    setJoystickButtonWhileHeld(driverStationJoystick, 9, new TurretTestCommand(turretSubsystem, -.1)); //turret L
    setJoystickButtonWhileHeld(driverStationJoystick, 10, new TurretTestCommand(turretSubsystem, .1)); //turret R
  }

  public double getLeftY() {
    if (driverStationJoystick.getRawAxis(1) >= .1 || driverStationJoystick.getRawAxis(1) <= -.1) {
      return driverStationJoystick.getRawAxis(1);
    } else {
      return 0;
    }
  }

  public double getLeftX() {
    return driverStationJoystick.getRawAxis(0);
  }

  public double getRightY() {
      return driverStationJoystick.getRawAxis(3);
  }

  public double getRightX() {
    return driverStationJoystick.getRawAxis(2);
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
