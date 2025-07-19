// Copyright (c) ielIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private static Elevator instance;

  private DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry positionLog = new DoubleLogEntry(log, "Elevator/Position");
  private final DoubleLogEntry targetLog = new DoubleLogEntry(log, "Elevator/Target");

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable elevatorTable = inst.getTable("Elevator");
  private final DoublePublisher positionTopic = elevatorTable.getDoubleTopic("Position").publish();
  private final DoublePublisher targetTopic = elevatorTable.getDoubleTopic("Target").publish();

  private final SparkMax masterMotor =
      new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
  private final SparkMax slaveMotor =
      new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

  // private boolean homed = true;
  // private Debouncer homeChecker = new Debouncer(0.3);

  public enum ElevatorTarget {
    HOME(80.0),
    L1(400.0),
    L2(452.0),
    L3(1176.0),
    L4(2372.0),
    STATION(1300.0);
    // TODO

    private final double height;

    private ElevatorTarget(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }

  private ElevatorTarget target = ElevatorTarget.HOME;

  public static synchronized Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    configureMotor();
  }

  private void configureMotor() {
    SparkMaxConfig masterConfig = new SparkMaxConfig();
    SparkMaxConfig slaveConfig = new SparkMaxConfig();

    masterConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    slaveConfig
        .follow(ElevatorConstants.kLeftMotorID, true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

    masterConfig.encoder.positionConversionFactor(ElevatorConstants.kUnitConversion);
    // .velocityConversionFactor(ElevatorConstants.kUnitConversion)

    masterConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD,
            ClosedLoopSlot.kSlot0)
     .outputRange(-0.5, 0.5);

    masterMotor.configure(
        masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    slaveMotor.configure(
        slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    final double position = getPosition();

    switch (target) {
      case HOME:
        //masterMotor.set(0);
        setPIDPosition(target.getHeight());
        break;
      default:
        setPIDPosition(target.getHeight());
        break;
    }

    positionLog.append(position);
    positionTopic.set(position);

    targetLog.append(target.getHeight());
    targetTopic.set(target.getHeight());
  }

  public void setVoltage(double voltage) {
    masterMotor.setVoltage(voltage);
  }

  public void stop() {
    masterMotor.set(0.0);
  }

  public void setPIDPosition(double targetPosition) {
    masterMotor
        .getClosedLoopController()
        .setReference(
            targetPosition,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            0);
    // TODO
  }

  public void setTarget(ElevatorTarget target) {
    if (this.target != target) {
      this.target = target;
    }
  }

  public ElevatorTarget getTarget() {
    return target;
  }

  public double getPosition() {
    return masterMotor.getEncoder().getPosition();
  }

  public double getRate() {
    return masterMotor.getEncoder().getVelocity() / 60 * ElevatorConstants.kUnitConversion;
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - target.getHeight()) <= ElevatorConstants.kAllowableError;
  }

  public Command runToTarget(ElevatorTarget target) {
    return run(() -> setTarget(target));
  }

  public Command setForgetTarget(ElevatorTarget target) {
    return runOnce(() -> setTarget(target));
  }

  public void resetEncoder() {
    masterMotor.getEncoder().setPosition(0.0);
  }

  public Command ampHoming() {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(-1.0),
        (interrupted) -> {
          if (!interrupted) {
            resetEncoder();
          }
        },
        () -> masterMotor.getOutputCurrent() > 20.0,
        this);
  }
}
