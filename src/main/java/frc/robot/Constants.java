// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = 51.3;
  // public static final Matter CHASSIS =
  //     new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  // public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_VELOCITY = 2.0;
  public static final double MAX_ANGULAR_VELOCITY = 120.0;

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.08;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    // public static final double TURN_CONSTANT = 6;
  }



  public static final class ElevatorConstants {
    public static final int kLeftMotorID = 62;
    public static final int kRightMotorID = 61;

    public static final double kP = 0.007;
    public static final double kI = 0.0;
    public static final double kD = 0.04;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;

    public static final double kMinOutput = 0.0;
    public static final double kMaxOutput = 2500.0;

    public static final double kUnitConversion = 0.0625 * 2 * Math.PI * 2.074 * 25.4 * 2.0;

    public static final double kAllowableError = 25.0;
  }

 

  
}
