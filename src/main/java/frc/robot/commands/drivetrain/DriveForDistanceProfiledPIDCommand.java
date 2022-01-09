// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForDistanceProfiledPIDCommand extends ProfiledPIDCommand {
  private DrivetrainSubsystem drivetrainSubsystem;
  private double encoderGoal;


  /** Creates a new DriveForDistanceProfiledPIDCommand. */
  public DriveForDistanceProfiledPIDCommand(DrivetrainSubsystem drivetrainSubsystem, double encoderGoal) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(DriveConstants.MAX_SPEED_INCHES_PER_SEC, DriveConstants.MAX_ACCEL_INCHES_PER_SEC2)),
        // This should return the measurement
        drivetrainSubsystem::getAverageEncoder,
        // This should return the goal (can also be a constant)
        encoderGoal,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          if(encoderGoal < 0){
            drivetrainSubsystem.tankDrive(-output, -output);
            System.out.println("on the move (backwards)");
          } else {
            drivetrainSubsystem.tankDrive(output, output);
            System.out.println("on the move (forwards)");
          }
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.encoderGoal = encoderGoal;
    drivetrainSubsystem.resetEncoders();
    getController().setTolerance(1.0);
  }

  @Override
  public void initialize() {
    super.initialize();
    drivetrainSubsystem.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
