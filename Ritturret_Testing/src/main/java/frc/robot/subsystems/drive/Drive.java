package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Drive extends SubsystemBase {
    private final CommandXboxController controller;

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public Drive(CommandXboxController controller, DriveIO io, GyroIO gyroIO){
        this.controller = controller;
        this.io = io;
        this.gyroIO = gyroIO;

        setDefaultCommand(runChassisSpeeds(this::getSpeedsFromController));
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        
    }

    public Command runChassisSpeeds(Supplier<ChassisSpeeds> speeds){
        return Commands.run(() -> io.requestSpeeds(speeds.get(), gyroInputs.heading));
    }

    private ChassisSpeeds getSpeedsFromController(){
        return new ChassisSpeeds(-controller.getLeftY(), -controller.getLeftX(), -controller.getRightX());
    }
    
}
