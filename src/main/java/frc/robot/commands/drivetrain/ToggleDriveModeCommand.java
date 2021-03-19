// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveMode;

public class ToggleDriveModeCommand extends CommandBase {

  private DrivetrainSubsystem drivetrainSubsystem;

  public ToggleDriveModeCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    if(drivetrainSubsystem.getDriveMode() == DriveMode.TANK){
      drivetrainSubsystem.setDriveMode(DriveMode.CHEEZY);
    } else {
      drivetrainSubsystem.setDriveMode(DriveMode.TANK);
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
