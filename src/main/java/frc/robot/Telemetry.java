package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class Telemetry {

  public Telemetry() {
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    DriverStation.startDataLog(log);
  }

  private DataLog log = DataLogManager.getLog();
  private DoubleLogEntry batteryLog = new DoubleLogEntry(log, "/hardware/batteryVoltage");

  public void periodic() {
    batteryLog.append(RobotController.getBatteryVoltage());
  }
}
