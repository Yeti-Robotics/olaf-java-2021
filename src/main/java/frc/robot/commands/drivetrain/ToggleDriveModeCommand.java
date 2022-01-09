// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveMode;

public class ToggleDriveModeCommand extends CommandBase {

  private DrivetrainSubsystem drivetrainSubsystem;
  private Joystick driverStationJoystick;

  public ToggleDriveModeCommand(DrivetrainSubsystem drivetrainSubsystem, Joystick driverStationJoystick) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.driverStationJoystick = driverStationJoystick;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    if(drivetrainSubsystem.getDriveMode() == DriveMode.TANK){
      drivetrainSubsystem.setDriveMode(DriveMode.CHEEZY);
      drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.cheezyDrive(driverStationJoystick.getRawAxis(1), driverStationJoystick.getRawAxis(2)), drivetrainSubsystem));

    } else {
      drivetrainSubsystem.setDriveMode(DriveMode.TANK);
      drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.tankDrive(driverStationJoystick.getRawAxis(1), driverStationJoystick.getRawAxis(3)), drivetrainSubsystem));
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  
    
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
