package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveIOHardware implements DriveIO {
    private VictorSPX[] motors = new VictorSPX[4];

    private MecanumDrive drive;

    public DriveIOHardware() {
        for (int i = 0; i < 4; i++) {
            motors[i] = new VictorSPX(i);
        }

        motors[0].setInverted(true);
        motors[1].setInverted(false);
        motors[2].setInverted(true);
        motors[3].setInverted(false);

        drive = new MecanumDrive(
                (output) -> motors[2].set(VictorSPXControlMode.PercentOutput, output),
                (output) -> motors[0].set(VictorSPXControlMode.PercentOutput, output),
                (output) -> motors[1].set(VictorSPXControlMode.PercentOutput, output),
                (output) -> motors[3].set(VictorSPXControlMode.PercentOutput, output));
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        
    }

    @Override
    public void requestSpeeds(ChassisSpeeds speeds, Rotation2d gyroAngle) {
        drive.driveCartesian(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, gyroAngle);
    }
    
    @Override
    public void requestSpeeds(ChassisSpeeds speeds) {
        drive.driveCartesian(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }
}
