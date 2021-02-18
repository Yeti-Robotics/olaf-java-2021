
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private CANSparkMax hoodspark;


  public HoodSubsystem() {
    hoodspark = new CANSparkMax(HoodConstants.HOOD_SPARK, MotorType.kBrushless);  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
