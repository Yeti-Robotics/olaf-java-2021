package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;;

public class HopperSubsystem extends SubsystemBase {
    private VictorSPX hopperVictor;

    public HopperSubsystem()    {
        hopperVictor = new VictorSPX(HopperConstants.HOPPER_VICTOR);
        hopperVictor.setInverted(true);
    }

    public void hopperIn()  {
        hopperVictor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_IN_SPEED);
    }
    public void hopperOut()  {
        hopperVictor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_OUT_SPEED);
    }

    public void hopperStop() {
        hopperVictor.set(ControlMode.PercentOutput, 0);   
    }
}


//One speed controller, Victor SPX, so it is modified for only one victor
