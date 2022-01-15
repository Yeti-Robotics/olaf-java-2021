// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnForAnglePIDCommand extends PIDCommand {

  private double gyroGoal;
  private DrivetrainSubsystem drivetrainSubsystem;
  /** Creates a new TurnForAnglePIDCommand. */
  public TurnForAnglePIDCommand(DrivetrainSubsystem drivetrainSubsystem, double gyroGoal) {
    super(
        // The controller that the command will use
        new PIDController(0.03, 0.002, 0.002),
        // new PIDController(.45*0.07, .54*0.07/1.06, 0.005),
        // This should return the measurement
        drivetrainSubsystem::getAngle,
        // This should return the setpoint (can also be a constant)
        gyroGoal,
        // This uses the output
        output -> {
        if(gyroGoal < 0){
          drivetrainSubsystem.tankDrive(-output, output);
          System.out.println("should b clockwise");
        }
        else {
          drivetrainSubsystem.tankDrive(-output, output);
          System.out.println("should b ccw");
        }
      }
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.gyroGoal = gyroGoal;
    getController().setTolerance(2.0);
  }

  @Override
    public void initialize() {
      super.initialize();
      drivetrainSubsystem.resetGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return getController().atSetpoint();
  }
}
