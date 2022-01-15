// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForDistancePIDCommand extends PIDCommand {
  private DrivetrainSubsystem drivetrainSubsystem;
  private double encoderGoal;
  /** Creates a new DriveForDistancePID. */
  public DriveForDistancePIDCommand(DrivetrainSubsystem drivetrainSubsystem, double encoderGoal) {
    super(
        // The controller that the command will use
        new PIDController(0.3, 0, 0),
        // This should return the measurement
        drivetrainSubsystem::getAverageEncoder,
        // This should return the setpoint (can also be a constant)
        encoderGoal,
        // This uses the output
        output -> {
          if(encoderGoal < 0){
            drivetrainSubsystem.tankDrive(-output, -output);
          } else {
            drivetrainSubsystem.tankDrive(output, output);
          }
        }
    );
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
