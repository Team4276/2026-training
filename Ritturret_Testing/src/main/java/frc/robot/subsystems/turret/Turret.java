package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    // TODO: Multithreading pog?
    private TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    public Turret(TurretIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    // TODO: figure out how tf to name ts
    public Command requestPositionCommand(DoubleSupplier position, DoubleSupplier velocity){
        return Commands.run(() -> {
            io.setSetpoint(position.getAsDouble(), velocity.getAsDouble());
        });
    }
}
