package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class GyroIOHardware implements GyroIO {
    private ADIS16470_IMU gyro;
    private double offset = 0.0;

    public GyroIOHardware(){
        gyro = new ADIS16470_IMU();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.heading = Rotation2d.fromDegrees(gyro.getAngle() - offset);
    }

    @Override
    public void zero() {
        offset = gyro.getAngle();
    }
}
