// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnForAngleCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private double leftPower, rightPower, gyroGoal;
  
  public TurnForAngleCommand(DrivetrainSubsystem driveSubsystem, double gyroGoal, double leftPower, double rightPower) {
    this.drivetrainSubsystem = driveSubsystem;
    this.gyroGoal = gyroGoal;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetGyro();
  }

  @Override
  public void execute(){
    if(gyroGoal < 0){
      drivetrainSubsystem.tankDrive(-leftPower, rightPower);
    } else {
      drivetrainSubsystem.tankDrive(leftPower, -rightPower);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(gyroGoal) <= drivetrainSubsystem.getAngle();
  }

  @Override 
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
  }
}
