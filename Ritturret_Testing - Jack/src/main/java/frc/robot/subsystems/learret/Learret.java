package frc.robot.subsystems.learret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Learret {
    private TalonFX motor = new TalonFX(5);
    
    VoltageOut voltageOutput = new VoltageOut(0.0);


    public Learret(){
        // Init shit here



    }
    public void idle(){
        motor.setVoltage(0);

    }
    public void update(double power){       //Range -1.0 -> 1.0
        motor.setVoltage(power*2);

    }

    public void setVoltage(double voltage){
        motor.setControl(voltageOutput.withOutput(voltage));
    }

    

}
