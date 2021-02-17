// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnForAngleCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private double leftPower, rightPower, gyroGoal;
  
  public TurnForAngleCommand(DriveSubsystem driveSubsystem, double gyroGoal, double leftPower, double rightPower) {
    this.driveSubsystem = driveSubsystem;
    this.gyroGoal = gyroGoal;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.resetGyro();
  }

  @Override
  public void execute(){
    if(gyroGoal < 0){
      driveSubsystem.tankDrive(-leftPower, rightPower);
    } else {
      driveSubsystem.tankDrive(leftPower, -rightPower);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(gyroGoal) <= driveSubsystem.getAngle();
  }

  @Override 
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
  }
}
