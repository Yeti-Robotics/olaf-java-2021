// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShiftingGearSubsystem;
import frc.robot.subsystems.ShiftingGearSubsystem.ShiftStatus;

public class DriveForDistanceCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private double distanceGoal;
  private double drivePower;
  /** Creates a new DriveForDistanceCommand. */

  public DriveForDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, double distanceGoal, double drivePower) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.drivePower = drivePower;
    this.distanceGoal = distanceGoal;
    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.tankDrive(drivePower, drivePower);
    System.out.println("Distance: " + drivetrainSubsystem.getLeftEncoder() + "; Encoder: " + drivetrainSubsystem.getRawEncoder() + "; gearing: " + ShiftingGearSubsystem.getShifterPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     System.out.println("drove forward");
      drivetrainSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceGoal <= this.drivetrainSubsystem.getAverageEncoder();
  }
}
