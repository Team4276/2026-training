package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.shooter.ShooterState;

public class AimCommands {
    public static Command alignTurret(RobotContainer robotContainer, Supplier<ShooterState> setpoint) {
        return Commands.runOnce(
                () -> {
                    var currSetpoint = setpoint.get();
                    robotContainer.getTurret().setSetpoint(currSetpoint.getTurretPosition(),
                            currSetpoint.getTurretFF());
                });
    }
}
