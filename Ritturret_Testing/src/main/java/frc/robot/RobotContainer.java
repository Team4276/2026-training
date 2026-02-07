// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimCommands;
import frc.robot.shooter.ShooterState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOHardware;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOHardware;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private Drive drive;
  private Turret turret;

  private Supplier<ShooterState> hubSetpointSupplier;

  private final CommandXboxController controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getType()) {
        case COMPBOT -> {
          drive = new Drive(controller, new DriveIOHardware(), new GyroIOHardware());
          turret = new Turret(new TurretIOHardware());
        }

        case SIMBOT -> {
          // Sim robot, instantiate physics sim IO implementations
          drive = new Drive(controller, new DriveIO() {
          }, new GyroIO() {

          });
          turret = new Turret(new TurretIO() {
          });
        }
      }
    }

    if (drive == null) {
      drive = new Drive(controller, new DriveIO() {
      }, new GyroIO() {

      });
    }

    // No-op implmentations for replay
    if (turret == null) {
      turret = new Turret(new TurretIO() {
      });
    }

    hubSetpointSupplier = ShooterState.getHubSetpointSupplier();

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controller
        .rightBumper()
        .toggleOnTrue(AimCommands.alignTurret(this, hubSetpointSupplier).repeatedly()
            .finallyDo(() -> AimCommands.alignTurret(this, () -> new ShooterState(0.0, 0.0))));

    controller
    .start()
    .onTrue(drive.zero());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }

  public Turret getTurret() {
    return turret;
  }
}
