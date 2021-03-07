package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
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
    private CANDigitalInput leftLimitSwitch;
    private CANDigitalInput rightLimitSwitch;
    private CANEncoder turretEncoder;

    public TurretSubsystem() {
        turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
        rightLimitSwitch = turretSpark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
        leftLimitSwitch = turretSpark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
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

    public double turretEncoderFromAngle(double angle){
        return ((angle/360)* TurretConstants.TURRET_GEAR_RATIO * TurretConstants.COUNTS_PER_REVOLUTION);
    }

    public boolean getPhysicalLimit(){
        return leftLimitSwitch.get() || rightLimitSwitch.get();
    }

    public boolean getReverseLimit() {
        return rightLimitSwitch.get();
    }

    public boolean getForwardLimit() {
        return leftLimitSwitch.get();
    }
}
