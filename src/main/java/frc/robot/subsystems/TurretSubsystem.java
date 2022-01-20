package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax turretSpark;
    private SparkMaxLimitSwitch leftLimitSwitch;
    private SparkMaxLimitSwitch rightLimitSwitch;
    private RelativeEncoder turretEncoder;

    public TurretSubsystem() {
        turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
        rightLimitSwitch = turretSpark.getReverseLimitSwitch(Type.kNormallyClosed);
        leftLimitSwitch = turretSpark.getForwardLimitSwitch(Type.kNormallyClosed);
        turretEncoder = turretSpark.getEncoder();
        turretSpark.setSoftLimit(SoftLimitDirection.kForward, (float)turretEncoderFromAngle(TurretConstants.TURRET_MAX_ANGLE));
        turretSpark.setSoftLimit(SoftLimitDirection.kReverse, (float)turretEncoderFromAngle(TurretConstants.TURRET_MIN_ANGLE));
        
        turretSpark.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
//        System.out.println("Turret Angle: " + turretAngleFromEncoder(getEncoder())+ "; Turret Encoder: "+ getEncoder());
    }

    public void moveTurret(double power) {
        turretSpark.set(power);
    }

    public void stopTurret() {
        turretSpark.set(0);
    }

    public void resetEncoder(){
        turretEncoder.setPosition(0);
    }

    public double getEncoder(){
        return turretEncoder.getPosition();
    }

    public double turretEncoderFromAngle(double angle){
        return TurretConstants.COUNTS_PER_DEGREE * angle;
    }

    public double turretAngleFromEncoder(double encoderValue){
        return encoderValue / TurretConstants.COUNTS_PER_DEGREE;
    }

    public boolean getPhysicalLimit(){
        return leftLimitSwitch.isPressed() || rightLimitSwitch.isPressed();
    }

    public boolean getReverseLimit() {
        return rightLimitSwitch.isPressed();
    }

    public boolean getForwardLimit() {
        return leftLimitSwitch.isPressed();
    }
}
