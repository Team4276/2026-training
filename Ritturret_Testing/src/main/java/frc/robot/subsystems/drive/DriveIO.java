package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
    }

    public default void updateInputs(DriveIOInputs inputs) {
    }

    public default void requestSpeeds(ChassisSpeeds speeds, Rotation2d gyroAngle){
    }

    public default void requestSpeeds(ChassisSpeeds speeds){
    }

}
