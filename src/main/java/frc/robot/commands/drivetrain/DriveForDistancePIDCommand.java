package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.Limelight;
import org.opencv.core.Mat;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;


public class TurnToTargetCommand extends PIDCommand {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private int ticks = 0;

    public TurnToTargetCommand(DrivetrainSubsystem drivetrainSubsystem) {
        super(
                new PIDController(Constants.kTurnP, Constants.kTurnI, Constants.kTurnD),
                // Close loop on heading
                //drivetrainSubsystem::drive,
                // Set reference to target
                //Limelight.getTx(),
                new DoubleSupplier() {
                    @Override
                    public double getAsDouble() {
                        return Limelight.getTx();
                    }
                },
                0.0,
                // Pipe output to turn robot
                //output -> drivetrainSubsystem.drive(0.5, 0.5),
                new DoubleConsumer() {
                    @Override
                    public void accept(double value) {
                      //  System.out.println("PID:" + value);
                        //drivetrainSubsystem.drive(-value, -value);
                        drivetrainSubsystem.driveWithMinPower(-value, -value, .2);
                    }
                },
                // Require the drive
                drivetrainSubsystem
        );
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(1.0);


        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        System.out.println("command accessed");
        if(Limelight.getTx()<0){
            drivetrainSubsystem.drive(-0.4,0.4);
            System.out.println("turning left");
        }else if(Limelight.getTx()>0){
            drivetrainSubsystem.drive(0.4,-0.4);
            System.out.println("turning right");
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println("done done done");
        return Math.abs(Limelight.getTx()) <= 2;
//        if (getController().atSetpoint()) {
//            ticks++;
//        }
//        else {
//            ticks = 0;
//        }
//        System.out.println("doop in finished :)");
//        return ticks > 20;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ended sir ^^");
        drivetrainSubsystem.drive(0,0);

    }
}