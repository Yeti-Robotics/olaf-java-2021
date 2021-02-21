package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Limelight;
import frc.robot.Constants;
import frc.robot.Constants.CalcConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax turretSpark;
    private double distance;
    private DigitalInput hallEffectSensor;

    public TurretSubsystem() {
        turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
        hallEffectSensor = new DigitalInput(TurretConstants.HALL_EFFECT_SENSOR_ID);
    }

    public double calcHoodAngle() {
        return Math.toDegrees(Math.asin(-CalcConstants.GRAVITY * distance) / ShooterConstants.SHOOT_1_SPEED);
    }

    public void periodic() {
        distance = Limelight.getCalculatedDistance();
    }

    public void moveTurret(double power) {
        turretSpark.set(power);
    }

    public void stopTurret() {
        turretSpark.set(0);
    }
}
