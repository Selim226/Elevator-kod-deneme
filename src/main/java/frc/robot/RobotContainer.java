// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.swerve.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController joystick = new CommandXboxController(0);

  public final Swerve swerve = Swerve.getInstance();
  public final Elevator elevator = Elevator.getInstance();
  // public final Climb climb = Climb.getInstance();

  public final Telemetry logger = new Telemetry();
  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    elevator.setDefaultCommand(elevator.runToTarget(ElevatorTarget.HOME));

    NamedCommands.registerCommand(
        "scoreL4",
        elevator
            .runToTarget(ElevatorTarget.L4).withTimeout(3)
            );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    SwerveInputStream driveAngularVelocity =
        SwerveInputStream.of(
                swerve.getSwerveDrive(),
                () -> joystick.getLeftY() * -1,
                () -> joystick.getLeftX() * -1)
            .withControllerRotationAxis(() -> -joystick.getRightX())
            .deadband(OperatorConstants.DEADBAND)
            .allianceRelativeControl(true);

    swerve.setDefaultCommand(swerve.driveCommand2(driveAngularVelocity));

    joystick
        .a()
        .whileTrue(
            elevator
                .runToTarget(ElevatorTarget.L1)
                );
    joystick
        .b()
        .whileTrue(
            elevator
                .runToTarget(ElevatorTarget.L2)
                );

    joystick
        .x()
        .whileTrue(
            elevator
                .runToTarget(ElevatorTarget.L3)
                );

    joystick
        .y()
        .whileTrue(
            elevator
                .runToTarget(ElevatorTarget.L4)
                );

    joystick
        .leftBumper()
        .whileTrue(
            elevator
                .runToTarget(ElevatorTarget.STATION)
                .alongWith(
                    ));





    joystick.povUp().onTrue(Commands.runOnce(swerve::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
