package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
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
    private DigitalInput leftLimitSwitch;
    private DigitalInput rightLimitSwitch;
    private CANEncoder turretEncoder;

    public TurretSubsystem() {
        turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
        hallEffectSensor = new DigitalInput(TurretConstants.HALL_EFFECT_SENSOR_ID);
        leftLimitSwitch = new DigitalInput(TurretConstants.LEFT_LIMIT_SWITCH_ID);
        rightLimitSwitch = new DigitalInput(TurretConstants.RIGHT_LIMIT_SWITCH_ID);
        turretEncoder = turretSpark.getEncoder();
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
    
    // i implemented something that i found in this github example but idk if it works. if not we can revert to what hank originally had
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Read%20Encoder%20Values/src/main/java/frc/robot/Robot.java
    public void resetEncoder(){
        turretEncoder.setPosition(0);
    }

    public double getEncoder(){
        return turretEncoder.getPosition();
    }

    public boolean getMagLimit(){
        return hallEffectSensor.get();
    }

    public boolean getPhysicalLimit(){
        if (leftLimitSwitch.get()){
            return true;
        } else if (rightLimitSwitch.get()){
            return true;
        } else {
            return false;
        }
    }
}
