package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorBetter extends VictorSPX{
    public VictorBetter(int id){
        super(id);
    }

    public void setVoltage(double voltage){
        super.set(ControlMode.PercentOutput, voltage);
    }
}
