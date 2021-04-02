
package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CalcConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private CANSparkMax hoodSpark;
  private CANEncoder hoodEncoder;
  private CANDigitalInput beamBreak;


  public HoodSubsystem() {
    hoodSpark = new CANSparkMax(HoodConstants.HOOD_SPARK, MotorType.kBrushless);  
    hoodEncoder = hoodSpark.getEncoder();
    hoodSpark.setInverted(true);
    beamBreak = hoodSpark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
    hoodSpark.setSoftLimit(SoftLimitDirection.kForward, (float)hoodEncoderFromAngle(HoodConstants.MAX_HOOD_ANGLE));
    hoodSpark.setSoftLimit(SoftLimitDirection.kReverse, (float)hoodEncoderFromAngle(0));

  }

  @Override
  public void periodic() {
  //  System.out.println("hood enc value: " + hoodSpark.getEncoder());
  }

  public void moveHood(double power) {
    hoodSpark.set(power);
  }

  public void stopHood() {
    hoodSpark.set(0);
  }

  public double hoodEncoderFromAngle(double angle){
    return HoodConstants.COUNTS_PER_DEGREE * angle;
  }

  public double hoodAngleFromEncoder(double encoderValue){
    return encoderValue / HoodConstants.COUNTS_PER_DEGREE;
  }

  public void resetEncoder(){
    hoodEncoder.setPosition(0);
}

  public double getEncoder(){
    return hoodEncoder.getPosition();
  }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

  public boolean getBeamBreak(){
    return beamBreak.get();
  }

  public double calcHoodAngle(double distance) {
    double angle;
    if(distance > 165.909){
      angle = 30;
    } else if(distance < 60.9){
      angle = 11.26896353;
    } else {
      //based on hood testing math; ax^2 + bx + c
      angle = -0.00176559 * Math.pow(distance, 2) + 0.585856 * distance - 17.7769;
    }
    return angle;
  }
}
