// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

  private static Swerve instance;

  public static synchronized Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    }
    return instance;
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  private final SwerveDrive swerveDrive;

  private Alliance mAlliance = Alliance.Red;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = inst.getTable("DriveState");
  private final StructPublisher<Pose2d> drivePose =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

  private DataLog log = DataLogManager.getLog();
  private final StructLogEntry<Pose2d> drivePoseLog =
      StructLogEntry.create(log, "DriveState/Pose", Pose2d.struct);

  public Swerve(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_VELOCITY);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true, false, 0.1); // TODO
    swerveDrive.setModuleEncoderAutoSynchronize(false, 3);
    configureAutoBuilder();

    // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyro)); TODO
  }

  @Override
  public void periodic() {
    DriverStation.getAlliance()
        .ifPresent(
            allianceColor -> {
              if (mAlliance != allianceColor) {
                mAlliance = allianceColor;
              }
            });

    drivePose.set(getPose());
    drivePoseLog.append(getPose());
  }

  public Alliance getAlliance() {
    return mAlliance;
  }

  public void configureAutoBuilder() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally
          // outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive
              // trains
              new PIDConstants(4.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(4.0, 0.0, 0.0)
              // Rotation PID constants
              ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
          );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          swerveDrive.drive(
              SwerveMath.scaleTranslation(
                  new Translation2d(
                      translationX.getAsDouble() * Constants.MAX_VELOCITY,
                      translationY.getAsDouble() * Constants.MAX_VELOCITY),
                  0.8),
              Math.pow(angularRotationX.getAsDouble(), 3) * Constants.MAX_ANGULAR_VELOCITY,
              true,
              true // TODO
              );
        });
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  public Command driveCommand2(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  velocity.get(), swerveDrive.getOdometryHeading()),
              true,
              new Translation2d());
        });
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();

    if (mAlliance == Alliance.Red) {
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    }
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);

    // TODO: Fix timestamp?? remove util thing and try
  }
}
