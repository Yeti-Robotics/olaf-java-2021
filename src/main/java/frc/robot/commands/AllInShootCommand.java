// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.PinchRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AllInShootCommand extends CommandBase {
  /** Creates a new AllInShootCommand. */
  private ShooterSubsystem shooterSubsystem;
  private HopperSubsystem hopperSubsystem;
  private PinchRollerSubsystem pinchRollerSubsystem;
  public AllInShootCommand(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem, PinchRollerSubsystem pinchRollerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.pinchRollerSubsystem = pinchRollerSubsystem;
    addRequirements(shooterSubsystem, hopperSubsystem, pinchRollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.shootFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(shooterSubsystem.getFlywheelRPM()- shooterSubsystem.setPoint) <= 50){
      hopperSubsystem.hopperIn();
      pinchRollerSubsystem.pinchIn();
    } else {
      hopperSubsystem.hopperStop();
      pinchRollerSubsystem.pinchStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.hopperStop();
    pinchRollerSubsystem.pinchStop();
    shooterSubsystem.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
