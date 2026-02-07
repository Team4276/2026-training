package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class CustomMotor extends VictorSPX {


    public CustomMotor(int arg0){
        super(arg0);
    }

    /** No voltage comp */
    public void setVoltage(double voltage){
        set(ControlMode.PercentOutput, voltage/12);
    }
}
