// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PinchRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;

public class AllInShootCommand extends CommandBase {
  /** Creates a new AllInShootCommand. */
  private ShooterSubsystem shooterSubsystem;
  private HopperSubsystem hopperSubsystem;
  private PinchRollerSubsystem pinchRollerSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private boolean readyToFire = false;
  public AllInShootCommand(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem, PinchRollerSubsystem pinchRollerSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.pinchRollerSubsystem = pinchRollerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(shooterSubsystem, hopperSubsystem, pinchRollerSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setSetPoint(shooterSubsystem.calcFlywheelRPM());
    shooterSubsystem.shootFlywheel();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Current RPM: "+ shooterSubsystem.getFlywheelRPM() +"; Calculated RPM: " + shooterSubsystem.setPoint);
      if (Math.abs(shooterSubsystem.getFlywheelRPM() - shooterSubsystem.setPoint) <= Constants.ShooterConstants.RPM_TOLERANCE){
        if(readyToFire){
          intakeSubsystem.intakeIn();
          hopperSubsystem.hopperIn();
          pinchRollerSubsystem.pinchIn();
        }
        else{
          try {
            wait(500);
            System.out.println("wait did the waiting :)");
          }
          catch (Exception e) {
            e.printStackTrace();
          }

          if (Math.abs(shooterSubsystem.getFlywheelRPM() - shooterSubsystem.setPoint) <= Constants.ShooterConstants.RPM_TOLERANCE){
            readyToFire = true;
          }
        }
      }
      else {
        intakeSubsystem.intakeStop();
        hopperSubsystem.hopperStop();
        pinchRollerSubsystem.pinchStop();
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.hopperStop();
    intakeSubsystem.intakeStop();
    pinchRollerSubsystem.pinchStop();
    shooterSubsystem.stopFlywheel();
    readyToFire = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
