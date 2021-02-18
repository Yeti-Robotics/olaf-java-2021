package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
//ToDo: create IntakeSubsystem 
//import frc.robot.subsystems.IntakeSubsystem;


public class StopDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    public StopDriveCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrainSubsystem.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
