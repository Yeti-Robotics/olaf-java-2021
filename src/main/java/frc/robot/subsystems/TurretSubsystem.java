package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Limelight;
import frc.robot.Constants.CalcConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

     // hood go brrr
     public PWMSparkMax hoodSpark;
     // turret go brrr
     public PWMSparkMax turretSpark;
     

     private double distance;

    public TurretSubsystem() {
        hoodSpark = new PWMSparkMax(TurretConstants.HOOD_SPARK);
        turretSpark = new PWMSparkMax(TurretConstants.TURRET_SPARK);
    }


    public double calcHoodAngle() {
        return Math.toDegrees(Math.asin( - CalcConstants.GRAVITY * distance) / ShooterConstants.SHOOT_1_SPEED);
    }
    public void periodic() {
        distance = Limelight.getCalculatedDistance();
    }

    public void setHoodAngle(double angle) {
        // insert fancy math here
    }

    public void moveHood(double power) {
        hoodSpark.set(power);
    }

    public void stopHood() {
        hoodSpark.set(0);
    }

    public void moveTurret(double power) {
        turretSpark.set(power);
    }

    public void stopTurret() {
        turretSpark.set(0);
    }
}


