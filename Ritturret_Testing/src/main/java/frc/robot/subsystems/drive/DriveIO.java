package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = true;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}
    
}
