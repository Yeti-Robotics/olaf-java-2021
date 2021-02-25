package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Limelight;
import frc.robot.Constants.CalcConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax turretSpark;
    private double distance;
    private DigitalInput hallEffectSensor;
    private DigitalInput limitSwitch;
    private double encoderOffset;

    public TurretSubsystem() {
        turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
        hallEffectSensor = new DigitalInput(TurretConstants.HALL_EFFECT_SENSOR_ID);
        limitSwitch = new DigitalInput(TurretConstants.LIMIT_SWITCH_ID);
        encoderOffset = 0.0;
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

    /*
        Couldn't find an actual way to reset SparkMAX, so I found this method on Chief Delphi from the Co-Founder of REV
        https://www.chiefdelphi.com/t/resetting-encoders-on-spark-max/344485

        Said to save the offset of the current encoder value and subtract it from subsequent reads. Not sure if I implemented it correctly
        but I tried ;)
    */
    public void resetEncoder(){
        encoderOffset = turretSpark.get();
    }

    public double getEncoder(){
        return turretSpark.get() - encoderOffset;
    }

    public boolean getMagLimit(){
        return hallEffectSensor.get();
    }

    public boolean getPhysicalLimit(){
        return limitSwitch.get();
    }
}
