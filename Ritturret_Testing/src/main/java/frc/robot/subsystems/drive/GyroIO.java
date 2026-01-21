package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d heading = Rotation2d.kZero;
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }
    
}
