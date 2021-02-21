
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private CANSparkMax hoodSpark;


  public HoodSubsystem() {
    hoodSpark = new CANSparkMax(HoodConstants.HOOD_SPARK, MotorType.kBrushless);  }

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

  public void setHoodAngle(double angle) {
    // insert fancy math here
  }

  public void setHoodMax(){
    // set hood to max up position
  }

  public void setHoodMin(){
    // set hood to min down position
  }
}
