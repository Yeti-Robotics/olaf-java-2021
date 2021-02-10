package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    private VictorSPX hopperVictor;

    public HopperSubsystem()    {
hopperVictor = new VictorSPX(Constants.HOPPER_VICTOR);
    }

    public void funnelIn()  {
        hopperVictor.set(ControlMode.PercentOutput, Constants.FUNNEL_IN_SPEED);
    }
    public void funnelOut()  {
        hopperVictor.set(ControlMode.PercentOutput, Constants.FUNNEL_OUTSPEED);

    public void funnelStop()    {
        hopperVictor.set(ControlMode.PercentOutput, 0);
        }    
    }
}


//One speed controller, Victor SPX, so it is modified for only one victor
