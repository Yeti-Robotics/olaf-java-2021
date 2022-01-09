// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShiftingGearSubsystem;
import frc.robot.subsystems.ShiftingGearSubsystem.ShiftStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForDistanceLowPIDCommand extends PIDCommand {
  private DrivetrainSubsystem drivetrainSubsystem;
  private double encoderGoal;
  private ShiftingGearSubsystem shiftingGearSubsystem;
  /** Creates a new DriveForDistancePID. */
  public DriveForDistanceLowPIDCommand(DrivetrainSubsystem drivetrainSubsystem, ShiftingGearSubsystem shiftingGearSubsystem, double encoderGoal) {
    super(
        // The controller that the command will use
        new PIDController(0.324, 0, 0.0),
        // This should return the measurement
        drivetrainSubsystem::getAverageEncoder,
        // This should return the setpoint (can also be a constant)
        encoderGoal,
        // This uses the output
        output -> {
          if(encoderGoal < 0){
            drivetrainSubsystem.tankDrive(-output, -output);
            System.out.println("on the move (backwards)");
          } else {
            drivetrainSubsystem.tankDrive(output, output);
            System.out.println("on the move (forwards)");
          }
        }
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shiftingGearSubsystem = shiftingGearSubsystem;
    this.encoderGoal = encoderGoal;
    drivetrainSubsystem.resetEncoders();
    getController().setTolerance(1.0);
  }
  
  @Override
  public void initialize() {
    super.initialize();
    drivetrainSubsystem.resetEncoders();
    shiftingGearSubsystem.shiftDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
