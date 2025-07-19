// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;


public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private final Swerve swerve;

 
  private final Elevator elevator;

  public Robot() {
    robotContainer = new RobotContainer();
    swerve = Swerve.getInstance();
    elevator = Elevator.getInstance();
 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.logger.periodic();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    swerve.setMotorBrake(true);
  }

  @Override
  public void autonomousInit() {
    DataLogManager.log("Starting Auto!");
    elevator.resetEncoder();

    // swerve.zeroGyro();

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    
  }

  @Override
  public void teleopInit() {
    DataLogManager.log("Starting Teleop!");

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }

   
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
