// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  public void setHeadingCorrection(boolean enable) {
    // Add logic to enable or disable heading correction
    // For example:
    if (enable) {
      // Enable heading correction logic
    } else {
      // Disable heading correction logic
    }
  }

  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  /**
   * Enable vision odometry updates while driving.
   */
  private final boolean visionDriveTest = false;
  /**
   * PhotonVision class to keep an accurate odometry.
   */
  private Vision vision;

  /**
   * Vision processing system for AprilTag odometry updates
   */
  private LimeLightStuff visionSystem;

  private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

  private boolean isFullyInitialized = false;
  private final Timer initializationTimer = new Timer();

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    initializationTimer.start();
    
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(180)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Wait for swerve drive to fully initialize
    Timer.delay(0.2); // Increased delay
    
    // Wait for gyro calibration
    waitForGyroCalibration();
    
    // Ensure initial pose is set safely after creation
    try {
      // Validate that all systems are ready
      if (validateSwerveReadiness()) {
        Pose2d initialPose = getPose();
        if (initialPose == null) {
          // Force initialization to origin if pose is null
          safeResetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));
          System.out.println("INITIALIZED: Robot pose to origin [0,0,0°] for PathPlanner compatibility");
        }
        isFullyInitialized = true;
      } else {
        System.err.println("Warning: Swerve drive not fully ready during construction");
      }
    } catch (Exception e) {
      System.err.println("Warning: Could not initialize pose during construction: " + e.getMessage());
      // Continue initialization - pose will be set later
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    
    if (visionDriveTest) {
      setupPhotonVision();
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();
    setupVisionSystem();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg,
        controllerCfg,
        Constants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
            Rotation2d.fromDegrees(180)));
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  /**
   * Setup the vision processing system
   */
  private void setupVisionSystem() {
    // Pass odometry reset callback to vision system
    visionSystem = new LimeLightStuff(this::processVisionMeasurement, this::resetOdometry);
    System.out.println("Vision system initialized with dual Limelight support and odometry reset capability");
  }

  private void processVisionMeasurement(LimeLightStuff.VisionMeasurement measurement) {

    var standardDeviations = edu.wpi.first.math.VecBuilder.fill(
        measurement.confidence * Constants.VisionConstants.XY_STD_DEV_FACTOR,
        measurement.confidence * Constants.VisionConstants.XY_STD_DEV_FACTOR,
        measurement.confidence * Constants.VisionConstants.ROTATION_STD_DEV_FACTOR
    );

    swerveDrive.addVisionMeasurement(
        measurement.pose,
        measurement.timestampSeconds,
        standardDeviations
    );

    // Log the measurement for debugging
    visionTab.add("Last Update Source", measurement.source);
    visionTab.add("Last Update X", measurement.pose.getX());
    visionTab.add("Last Update Y", measurement.pose.getY());
    visionTab.add("Last Update Rotation", measurement.pose.getRotation().getDegrees());
    visionTab.add("Last Update Timestamp", measurement.timestampSeconds);
    visionTab.add("Last Update Tag Count", measurement.tagCount);
  }

  @Override
  public void periodic() {

    // Mark as fully initialized after first successful periodic cycle
    if (!isFullyInitialized && initializationTimer.hasElapsed(1.0)) {
      if (validateSwerveReadiness()) {
        isFullyInitialized = true;
        System.out.println("Swerve drive fully initialized after " + initializationTimer.get() + " seconds");
      }
    }

    // Update odometry frequently
    swerveDrive.updateOdometry();

    if (visionSystem != null) {
      double yawRadians = getHeading().getRadians();
      double yawRateRadPerSec = getRobotVelocity().omegaRadiansPerSecond;
      visionSystem.updateRobotOrientation(yawRadians, yawRateRadPerSec);
      visionSystem.processVisionMeasurements();
    }

    // Get current robot pose and send to SmartDashboard
    Pose2d currentPose = getPose();
    SmartDashboard.putNumber("Robot X Position", currentPose.getX());
    SmartDashboard.putNumber("Robot Y Position", currentPose.getY());
    SmartDashboard.putNumber("Robot Rotation (deg)", currentPose.getRotation().getDegrees());
    SmartDashboard.putString("Robot Pose", String.format("(%.2f, %.2f, %.1f°)",
        currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()));

    // Additional pose information
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());

    // Robot velocity information
    ChassisSpeeds robotVelocity = getRobotVelocity();
    SmartDashboard.putNumber("Robot Velocity X", robotVelocity.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Velocity Y", robotVelocity.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Angular Velocity", Units.radiansToDegrees(robotVelocity.omegaRadiansPerSecond));

    //  Shuffleboard.getTab("Drivetrain").add("Gyro Heading", swerveDrive.getPose().getRotation().getDegrees());
    //  Shuffleboard.getTab("Drivetrain").add("Robot X Position", swerveDrive.getPose().getX());
    // Shuffleboard.getTab("Drivetrain").add("Robot Y Position", swerveDrive.getPose().getY());
  }

  /**
   * Get the vision system for direct access if needed
   */
  public LimeLightStuff getVisionSystem() {
    return visionSystem;
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      
      AutoBuilder.configure(
          this::getPose,
          this::safePathPlannerReset, // Use safe reset for PathPlanner
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(7.0, 0, 0),
              new PIDConstants(7.0, 0, 0)
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      );

    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("CRITICAL: PathPlanner configuration failed: " + e.getMessage());
    }

    try {
      PathfindingCommand.warmupCommand().schedule();
    } catch (Exception e) {
      System.out.println("WARNING: PathPlanner warmup failed: " + e.getMessage());
    }
  }

  /**
   * Get the path follower with events.
   */
  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 3.0,   // Reduced acceleration
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(360) // Reduced angular velocity
    );
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
        swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint
        = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
        swerveDrive.getStates(),
        DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);

        });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, visionDriveTest),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
            distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
              translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
              translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 2);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.

  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   */
  public Pose2d getPose() {
    try {
      if (!isFullyInitialized) {
        System.out.println("Warning: Getting pose before full initialization");
        return new Pose2d(); // Return origin as fallback
      }
      return swerveDrive.getPose();
    } catch (Exception e) {
      System.err.println("Warning: Could not get pose: " + e.getMessage());
      return new Pose2d(); // Return origin as fallback
    }
  }

  /**
   * Safely reset odometry with null checking
   */
  private void safeResetOdometry(Pose2d pose) {
    try {
      if (swerveDrive != null) {
        swerveDrive.resetOdometry(pose);
      }
    } catch (Exception e) {
      System.err.println("Warning: Could not reset odometry safely: " + e.getMessage());
    }
  }

  /**
   * Resets odometry to the given pose.
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    try {
      if (swerveDrive == null) {
        System.err.println("Error: SwerveDrive not initialized, cannot reset odometry");
        return;
      }
      
      if (!isFullyInitialized) {
        System.err.println("Warning: Attempting odometry reset before full initialization");
        // Try to validate readiness now
        if (!validateSwerveReadiness()) {
          System.err.println("Error: Cannot reset odometry - system not ready");
          return;
        }
      }
      
      // Validate pose input
      if (initialHolonomicPose == null) {
        initialHolonomicPose = new Pose2d();
        System.out.println("Warning: Null pose provided, using origin");
      }
      
      // Additional validation - ensure pose is reasonable
      double x = initialHolonomicPose.getX();
      double y = initialHolonomicPose.getY();
      if (Double.isNaN(x) || Double.isNaN(y) || Double.isInfinite(x) || Double.isInfinite(y)) {
        System.err.println("Error: Invalid pose coordinates, using origin");
        initialHolonomicPose = new Pose2d();
      }
      
      System.out.println("Attempting to reset odometry to: " + initialHolonomicPose);
      
      swerveDrive.resetOdometry(initialHolonomicPose);
      System.out.println("Odometry reset successful");
      
    } catch (Exception e) {
      System.err.println("Error resetting odometry: " + e.getMessage());
      e.printStackTrace();
      
      // Try alternative initialization approach
      System.out.println("Attempting alternative odometry initialization...");
      Timer.delay(0.1);
      
      try {
        // Force re-validation
        if (validateSwerveReadiness()) {
          swerveDrive.resetOdometry(new Pose2d());
          System.out.println("Alternative odometry reset to origin successful");
        } else {
          System.err.println("Critical: Alternative odometry reset also failed - system not ready");
        }
      } catch (Exception e2) {
        System.err.println("Critical: All odometry reset attempts failed: " + e2.getMessage());
        e2.printStackTrace();
      }
    }
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 180.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();

  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  /**
   * Safe odometry reset specifically for PathPlanner with additional error handling
   */
  private void safePathPlannerReset(Pose2d pose) {
    try {
      // Extra validation for PathPlanner
      if (!isFullyInitialized) {
        System.err.println("PathPlanner Error: Swerve drive not fully initialized");
        
        // Try to force initialization
        double waitStart = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - waitStart < 2.0) { // Wait up to 2 seconds
          if (validateSwerveReadiness()) {
            isFullyInitialized = true;
            System.out.println("PathPlanner: Swerve drive initialization completed");
            break;
          }
          Timer.delay(0.1);
        }
        
        if (!isFullyInitialized) {
          System.err.println("PathPlanner Error: Could not complete swerve initialization in time");
          return;
        }
      }
      
      // Validate inputs
      if (pose == null) {
        pose = new Pose2d();
        System.out.println("PathPlanner: Null pose provided, using origin");
      }
      
      // Check if systems are ready
      if (swerveDrive == null) {
        System.err.println("PathPlanner Error: SwerveDrive not initialized");
        return;
      }
      
      // Add debug logging
      System.out.println("PathPlanner: Attempting to reset odometry to " + pose);
      
      // Ensure all components are ready before reset
      Timer.delay(0.05); // Slightly longer delay for PathPlanner
      
      resetOdometry(pose);
      
    } catch (Exception e) {
      System.err.println("PathPlanner odometry reset failed: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Wait for gyro to finish calibration
   */
  private void waitForGyroCalibration() {
    System.out.println("Waiting for gyro calibration...");
    
    // Wait up to 3 seconds for gyro to be ready
    double startTime = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() - startTime < 3.0) {
      try {
        // Try to get a gyro reading - this will throw if not ready
        swerveDrive.getYaw();
        System.out.println("Gyro calibration complete");
        Timer.delay(0.1); // Small additional delay after calibration
        return;
      } catch (Exception e) {
        // Gyro not ready yet, wait a bit more
        Timer.delay(0.1);
      }
    }
    
    System.err.println("Warning: Gyro calibration may not be complete after 3 seconds");
  }

  /**
   * Validate that swerve drive is ready for operation
   */
  private boolean validateSwerveReadiness() {
    try {
      // Check if we can get basic readings without exceptions
      swerveDrive.getYaw();
      swerveDrive.getStates();
      swerveDrive.getPose();
      
      // Check that modules are responding
      var modules = swerveDrive.getModules();
      if (modules == null || modules.length == 0) {
        return false;
      }
      
      System.out.println("Swerve drive validation successful");
      return true;
    } catch (Exception e) {
      System.err.println("Swerve drive validation failed: " + e.getMessage());
      return false;
    }
  }
}
