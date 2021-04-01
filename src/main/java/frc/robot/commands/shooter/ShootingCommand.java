/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingCommand extends CommandBase {
  /**
   * Creates a new TestShootingCommand.
   */
  private final ShooterSubsystem shooterSubsystem;
  public ShootingCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setSetPoint(shooterSubsystem.calcFlywheelRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Current RPM: "+ shooterSubsystem.getFlywheelRPM() +"; Calculated RPM: " + shooterSubsystem.setPoint);
    shooterSubsystem.shootFlywheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooterSubsystem.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(shooterSubsystem.getFlywheelRPM()- shooterSubsystem.setPoint) <= Constants.ShooterConstants.RPM_TOLERANCE){

      return true;
    } else {
      return false;
    }
  }
}
