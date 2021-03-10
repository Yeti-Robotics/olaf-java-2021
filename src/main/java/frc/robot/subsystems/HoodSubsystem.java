
package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

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
    hoodSpark.setSoftLimit(SoftLimitDirection.kForward, (float)hoodEncoderFromAngle(HoodConstants.FORWARD_SOFT_LIMIT));
  }

  @Override
  public void periodic() {
//    System.out.println("hood enc value: " + hoodSpark.getEncoder());
//    System.out.println("beam break reverse value: " + getBeamBreak());
  }

  public void moveHood(double power) {
    hoodSpark.set(power);
  }

  public void stopHood() {
    hoodSpark.set(0);
  }

  public double hoodEncoderFromAngle(double angle) {
    // insert fancy math here
    return ((angle/360.0)* HoodConstants.HOOD_GEAR_RATIO * HoodConstants.COUNTS_PER_REVOLUTION);
  }

  public double getHoodEncoder(){
    return hoodEncoder.getPosition();
  }

  public boolean getBeamBreak(){
    return beamBreak.get();
  }

}
