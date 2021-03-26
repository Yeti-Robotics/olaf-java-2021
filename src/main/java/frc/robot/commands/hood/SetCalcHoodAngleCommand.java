package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.utils.Limelight;


public class SetCalcHoodAngleCommand extends CommandBase {
    private final HoodSubsystem hoodSubsystem;
    private double encoderGoal;
    private double power;
    public SetCalcHoodAngleCommand(HoodSubsystem hoodSubsystem, double power) {
        this.hoodSubsystem = hoodSubsystem;
        this.power = Math.abs(power);
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        encoderGoal = hoodSubsystem.hoodEncoderFromAngle(hoodSubsystem.calcHoodAngle(Limelight.getHorDistance()));
        if (encoderGoal < hoodSubsystem.getEncoder()){
            power = -power;
        }
    }

    @Override
    public void execute() {
        System.out.println("Current Angle: " + hoodSubsystem.hoodAngleFromEncoder(hoodSubsystem.getEncoder()) +"; Calculated Hood Angle: " + hoodSubsystem.calcHoodAngle(Limelight.getHorDistance()));
        hoodSubsystem.moveHood(power);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(encoderGoal - hoodSubsystem.getEncoder()) <= Constants.HoodConstants.HOOD_ANGLE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.stopHood();
    }
}
