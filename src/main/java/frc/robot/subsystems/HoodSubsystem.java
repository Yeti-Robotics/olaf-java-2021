
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private CANSparkMax hoodSpark;
  private CANEncoder hoodEncoder;

  public HoodSubsystem() {
    hoodSpark = new CANSparkMax(HoodConstants.HOOD_SPARK, MotorType.kBrushless);  
    hoodEncoder = hoodSpark.getEncoder();
  }

  @Override
  public void periodic() {
    System.out.println("hood enc value: " + hoodSpark.getEncoder());
  }

  public void moveHood(double power) {
    hoodSpark.set(power);
  }

  public void stopHood() {
    hoodSpark.set(0);
  }

  public void resetHood(double power){
    hoodSpark.set(-power);
  }

  public double encoderFromAngle(double angle) {
    // insert fancy math here
    return ((angle/360)* HoodConstants.HOOD_GEAR_RATIO * HoodConstants.COUNTS_PER_REVOLUTION);
  }

  public double getHoodEncoder(){
    return hoodEncoder.getPosition();
  }

}
