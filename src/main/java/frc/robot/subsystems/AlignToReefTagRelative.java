package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ElevatorConstants; // ADDED: Import ElevatorConstants
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = 1;
  private boolean isAlgaeKickingMode = false; // NEW: Track if this is algae kicking mode
  
  // Dual Limelight support with alignment-based switching
  private String currentLimelightName;
  private String primaryLimelight;   // Will be set based on alignment side
  private String secondaryLimelight; // Will be set based on alignment side
  private Timer switchDelayTimer;
  private Timer alignmentProgressTimer;
  private static final double SWITCH_DELAY = 0.5;
  private static final double ALIGNMENT_PROGRESS_TIMEOUT = 3.0;
  
  // Track alignment progress for each camera
  private double lastErrorDistance = Double.MAX_VALUE;
  private boolean makingProgress = true;

  // NEW: Driver override detection
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier rightXSupplier;
  private static final double DRIVER_INPUT_THRESHOLD = 0.1;
  private boolean driverOverride = false;

  // NEW: Elevator reference for motor control
  private ElevatorSubsystem elevator;
  private Timer shooterTimer;
  private boolean shooterActivated = false;
  private boolean algaeActivated = false; // NEW: Track algae kicker state separately
  private boolean alignmentCompleted = false;
  private static final double SHOOTER_RUN_TIME = 0.5; // CHANGED: Back to continuous run time

  // NEW: Add state tracking to prevent excessive logging and commands
  private double lastElevatorTagID = -1;
  private boolean elevatorPositionSet = false;

  // NEW: Constructor with driver input suppliers and elevator
  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase, boolean forceLeftCameraPriority, boolean isAlgaeKickingMode, 
                                DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, ElevatorSubsystem elevator) {
    xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);
    yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
    rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0.0);
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.isAlgaeKickingMode = isAlgaeKickingMode;
    this.switchDelayTimer = new Timer();
    this.alignmentProgressTimer = new Timer();
    this.shooterTimer = new Timer();
    
    // NEW: Store driver input suppliers
    this.leftYSupplier = leftY;
    this.leftXSupplier = leftX;
    this.rightXSupplier = rightX;
    this.elevator = elevator;
    
    // ALGAE KICKING MODE: Always prioritize RIGHT camera for stable alignment
    if (isAlgaeKickingMode) {
      primaryLimelight = "limelight-right";   // Always prioritize RIGHT camera for algae kicking
      secondaryLimelight = "limelight-left";  // Fallback to left camera
    } else if (forceLeftCameraPriority) {
      primaryLimelight = "limelight-left";    // Force left camera priority for other modes
      secondaryLimelight = "limelight-right"; // Fallback to right camera
    } else {
      // NORMAL: Set camera priorities to match alignment side
      if (isRightScore) {
        primaryLimelight = "limelight-right";
        secondaryLimelight = "limelight-left";
      } else {
        primaryLimelight = "limelight-left";
        secondaryLimelight = "limelight-right";
      }
    }
    
    addRequirements(drivebase);
  }

  // Keep existing constructors for backward compatibility - modified to work without driver input
  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    this(isRightScore, drivebase, false, false, () -> 0.0, () -> 0.0, () -> 0.0, null);
  }

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase, boolean forceLeftCameraPriority) {
    this(isRightScore, drivebase, forceLeftCameraPriority, false, () -> 0.0, () -> 0.0, () -> 0.0, null);
  }

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase, boolean forceLeftCameraPriority, boolean isAlgaeKickingMode) {
    this(isRightScore, drivebase, forceLeftCameraPriority, isAlgaeKickingMode, () -> 0.0, () -> 0.0, () -> 0.0, null);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.switchDelayTimer.start();
    this.alignmentProgressTimer.start();

    currentLimelightName = getBestInitialCamera();
    updateControllerSetpoints();
    tagID = getBestTagID();
    lastErrorDistance = Double.MAX_VALUE;
    makingProgress = true;
    driverOverride = false;
    shooterActivated = false;
    alignmentCompleted = false;
    
    // NEW: Reset elevator state tracking
    lastElevatorTagID = -1;
    elevatorPositionSet = false;
    
    String modeDescription = isAlgaeKickingMode ? "ALGAE KICKING" : "NORMAL";
    System.out.println("Smart Dual Limelight alignment started - Mode: " + modeDescription + 
                      ", Alignment Side: " + (isRightScore ? "RIGHT" : "LEFT") + 
                      ", Primary Camera: " + primaryLimelight + ", Secondary Camera: " + secondaryLimelight + 
                      ", Starting Camera: " + currentLimelightName);
  }

  @Override
  public void execute() {
    // ENHANCED: Check for driver override with algae-specific messaging
    if (checkDriverOverride()) {
      driverOverride = true;
      drivebase.drive(new Translation2d(), 0, false);
      
      // Enhanced logging for algae mode
      if (isAlgaeKickingMode) {
        System.out.println("ALGAE ALIGNMENT CANCELLED: Driver override detected - stopping algae alignment and kicker");
      } else {
        System.out.println("ALIGNMENT STOPPED: Driver override detected");
      }
      return;
    }

    // FIXED: Add missing alignment completion detection
    boolean isCurrentlyAligned = rotController.atSetpoint() &&
                                yController.atSetpoint() &&
                                xController.atSetpoint();

    // FIXED: Detect when alignment completes for the first time
    if (isCurrentlyAligned && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME) && !alignmentCompleted) {
      alignmentCompleted = true;
      
      if (isAlgaeKickingMode) {
        System.out.println("ALGAE ALIGNMENT ACHIEVED: Starting " + SHOOTER_RUN_TIME + " second continuous algae kicker");
      } else {
        System.out.println("ALIGNMENT ACHIEVED: Starting " + SHOOTER_RUN_TIME + " second continuous shooter");
      }
    }

    // FIXED: Handle motor activation based on alignment mode
    if (alignmentCompleted && !shooterActivated && !algaeActivated && elevator != null) {
      // Start appropriate motor and timer based on mode
      shooterTimer.restart();
      
      if (isAlgaeKickingMode) {
        // ENHANCED: Activate algae kicker AND lock shooter position for algae alignment mode
        elevator.lockShooterPosition(); // NEW: Lock shooter in brake mode (holds position)
        elevator.setAlgaeKickerSpeed(ElevatorConstants.ALGAE_KICKER_ON);
      
        algaeActivated = true;
        System.out.println("ALGAE KICKER STARTED: Running continuously for " + SHOOTER_RUN_TIME + " seconds");
        System.out.println("SHOOTER LOCKED: Position held in brake mode during algae operations");
      } else {
        // Activate shooter for normal alignment mode
        elevator.setShooterSpeed(-1.0); // Full power reverse for shooter
        shooterActivated = true;
        System.out.println("SHOOTER STARTED: Running continuously for " + SHOOTER_RUN_TIME + " seconds");
      }
    }

    // FIXED: Maintain continuous motor operation for full duration based on mode
    if ((shooterActivated || algaeActivated) && elevator != null) {
      if (shooterTimer.get() < SHOOTER_RUN_TIME) {
        // CONTINUOUSLY send motor command every cycle for the full duration
        if (isAlgaeKickingMode && algaeActivated) {
          elevator.setAlgaeKickerSpeed(ElevatorConstants.ALGAE_KICKER_ON); // Keep algae kicker running
          // Shooter remains locked in brake mode - no need to continuously call lockShooterPosition()
        } else if (!isAlgaeKickingMode && shooterActivated) {
          elevator.setShooterSpeed(-1.0); // Keep shooter running at full power
        }
      } else {
        // Timer elapsed - stop appropriate motor and mark sequence complete
        if (isAlgaeKickingMode && algaeActivated) {
          elevator.setAlgaeKickerSpeed(ElevatorConstants.ALGAE_KICKER_STOP); // Stop algae kicker
          elevator.releaseShooterLock(); // NEW: Release shooter from brake mode lock
          System.out.println("ALGAE KICKER SEQUENCE COMPLETE: Stopped after " + SHOOTER_RUN_TIME + " seconds of continuous operation");
          System.out.println("SHOOTER RELEASED: No longer locked in brake mode");
        } else if (!isAlgaeKickingMode && shooterActivated) {
          elevator.setShooterSpeed(0.0); // Stop shooter
          System.out.println("SHOOTER SEQUENCE COMPLETE: Stopped after " + SHOOTER_RUN_TIME + " seconds of continuous operation");
        }
        // Command will end in isFinished() method after this
      }
      
      // During continuous motor operation, don't do alignment - just maintain current position
      drivebase.drive(new Translation2d(), 0, false);
      
      // FIXED: Add SmartDashboard debugging during continuous motor operation
      updateMotorDebugInfo();
      return; // Skip alignment while motor is running
    }

    // Normal alignment execution - only run when not running motors
    updateCurrentLimelightSmart();
    
    double detectedTagID = LimelightHelpers.getFiducialID(currentLimelightName);
    
    if (LimelightHelpers.getTV(currentLimelightName) && detectedTagID == tagID) {
      this.dontSeeTagTimer.reset();
      
      double[] positions = LimelightHelpers.getBotPose_TargetSpace(currentLimelightName);

      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = rotController.calculate(positions[4]);
      System.out.println(positions[4]+ "s" + rotController.getSetpoint());
      if(positions[4] > rotController.getSetpoint())
      {
        rotValue = -.7;
      }
      else
      {
        rotValue = .7;
      }

      double currentErrorDistance = Math.sqrt(
          Math.pow(positions[2], 2) + 
          Math.pow(positions[0], 2) + 
          Math.pow(positions[4], 2)
      );
      
      updateAlignmentProgress(currentErrorDistance);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      // FIXED: Reset stopTimer only if not aligned (allows completion detection)
      if (!isCurrentlyAligned) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
      makingProgress = false;
      // FIXED: Reset alignment completion if we lose the tag
      if (alignmentCompleted && !shooterActivated && !algaeActivated) {
        alignmentCompleted = false;
        System.out.println("ALIGNMENT LOST: Tag no longer visible, resetting completion status");
      }
    }
    
    // FIXED: Add continuous SmartDashboard debugging
    updateMotorDebugInfo();
  }

  // ENHANCED: Better driver override detection with algae-specific logging
  private boolean checkDriverOverride() {
    if (leftYSupplier == null || leftXSupplier == null || rightXSupplier == null) {
      return false; // No input suppliers provided
    }

    double leftY = Math.abs(leftYSupplier.getAsDouble());
    double leftX = Math.abs(leftXSupplier.getAsDouble());
    double rightX = Math.abs(rightXSupplier.getAsDouble());

    boolean override = leftY > DRIVER_INPUT_THRESHOLD || 
                      leftX > DRIVER_INPUT_THRESHOLD || 
                      rightX > DRIVER_INPUT_THRESHOLD;

    // Enhanced debug info for algae mode
    if (override && isAlgaeKickingMode) {
      SmartDashboard.putBoolean("Algae Override Active", true);
      SmartDashboard.putNumber("Override Left Y", leftY);
      SmartDashboard.putNumber("Override Left X", leftX);
      SmartDashboard.putNumber("Override Right X", rightX);
      SmartDashboard.putNumber("Override Threshold", DRIVER_INPUT_THRESHOLD);
    } else if (!override && isAlgaeKickingMode) {
      SmartDashboard.putBoolean("Algae Override Active", false);
    }

    return override;
  }

  private void updateCurrentLimelightSmart() {
    // ALGAE MODE: Always prefer RIGHT camera, fall back to left if right unavailable
    // NORMAL MODE: Always prefer LEFT camera, fall back to right if left unavailable
    String rightLimelight = "limelight-right";
    String leftLimelight = "limelight-left";
    
    boolean rightHasTarget = LimelightHelpers.getTV(rightLimelight);
    boolean leftHasTarget = LimelightHelpers.getTV(leftLimelight);
    
    String targetCamera = currentLimelightName;
    String switchReason = "";
    boolean shouldSwitch = false;
    
    if (isAlgaeKickingMode) {
      // ALGAE MODE: Use RIGHT camera if available, otherwise use left camera
      // If currently using left camera but right camera has target, switch to right
      if (currentLimelightName.equals(leftLimelight) && rightHasTarget) {
        shouldSwitch = true;
        targetCamera = rightLimelight;
        switchReason = "ALGAE MODE: Switching to preferred right camera";
      }
      // If currently using right camera but it loses target and left has target, switch to left
      else if (currentLimelightName.equals(rightLimelight) && !rightHasTarget && leftHasTarget) {
        shouldSwitch = true;
        targetCamera = leftLimelight;
        switchReason = "ALGAE MODE: Right camera lost target, switching to left camera";
      }
    } else {
      // NORMAL MODE: Use LEFT camera if available, otherwise use right camera
      // If currently using right camera but left camera has target, switch to left
      if (currentLimelightName.equals(rightLimelight) && leftHasTarget) {
        shouldSwitch = true;
        targetCamera = leftLimelight;
        switchReason = "NORMAL MODE: Switching to preferred left camera";
      }
      // If currently using left camera but it loses target and right has target, switch to right
      else if (currentLimelightName.equals(leftLimelight) && !leftHasTarget && rightHasTarget) {
        shouldSwitch = true;
        targetCamera = rightLimelight;
        switchReason = "NORMAL MODE: Left camera lost target, switching to right camera";
      }
    }
    
    if (shouldSwitch && switchDelayTimer.hasElapsed(SWITCH_DELAY)) {
      currentLimelightName = targetCamera;
      updateControllerSetpoints();
      switchDelayTimer.restart();
      alignmentProgressTimer.restart();
      lastErrorDistance = Double.MAX_VALUE;
      makingProgress = true;
      System.out.println("CAMERA SWITCH: " + switchReason + " -> " + targetCamera);
    }
  }

  private void updateControllerSetpoints() {
    if (currentLimelightName.equals("limelight-right")) {
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_RIGHT);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT_RIGHT);
      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT_RIGHT);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT_RIGHT);
      
      // SIMPLE: Always use algae offset for Y when in algae mode, otherwise use normal alignment logic
      if (isAlgaeKickingMode) {
        yController.setSetpoint(VisionConstants.Y_OFFSET_ALGAE_KICKER);
      } else if (isRightScore) {
        yController.setSetpoint(Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT));
      } else {
        yController.setSetpoint(-Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT));
      }
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT_RIGHT);
    } else {
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_LEFT);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT_LEFT);
      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT_LEFT);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT_LEFT);
      
      // SIMPLE: Always use algae offset for Y when in algae mode, otherwise use normal alignment logic
      if (isAlgaeKickingMode) {
        yController.setSetpoint(VisionConstants.Y_OFFSET_ALGAE_KICKER);
      } else if (isRightScore) {
        double baseYSetpoint = -Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
        double specialOffset = VisionConstants.Y_OFFSET_RIGHT_ALIGN_ON_LEFT_CAMERA;
        yController.setSetpoint(baseYSetpoint + specialOffset);
      } else {
        yController.setSetpoint(Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT));
      }
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT_LEFT);
    }
  }

  /**
   * Get the best initial camera - algae mode prefers RIGHT, normal mode prefers LEFT
   */
  private String getBestInitialCamera() {
    if (isAlgaeKickingMode) {
      // ALGAE MODE: Always try RIGHT camera first
      if (LimelightHelpers.getTV("limelight-right")) {
        return "limelight-right";
      } else if (LimelightHelpers.getTV("limelight-left")) {
        return "limelight-left";
      } else {
        return "limelight-right"; // Default to right camera even if no target
      }
    } else {
      // NORMAL MODE: Always try LEFT camera first
      if (LimelightHelpers.getTV("limelight-left")) {
        return "limelight-left";
      } else if (LimelightHelpers.getTV("limelight-right")) {
        return "limelight-right";
      } else {
        return "limelight-left"; // Default to left camera even if no target
      }
    }
  }

  /**
   * Get any available tag - algae mode prefers RIGHT camera, normal mode prefers LEFT camera
   */
  private double getBestTagID() {
    if (isAlgaeKickingMode) {
      // ALGAE MODE: Try RIGHT camera first
      if (LimelightHelpers.getTV("limelight-right")) {
        double rightTagID = LimelightHelpers.getFiducialID("limelight-right");
        if (rightTagID > 0) {
          System.out.println("ALGAE MODE: Selected right camera tag: " + rightTagID);
          return rightTagID;
        }
      }
      
      // Fall back to left camera
      if (LimelightHelpers.getTV("limelight-left")) {
        double leftTagID = LimelightHelpers.getFiducialID("limelight-left");
        if (leftTagID > 0) {
          System.out.println("ALGAE MODE: Selected left camera tag: " + leftTagID);
          return leftTagID;
        }
      }
    } else {
      // NORMAL MODE: Try LEFT camera first
      if (LimelightHelpers.getTV("limelight-left")) {
        double leftTagID = LimelightHelpers.getFiducialID("limelight-left");
        if (leftTagID > 0) {
          System.out.println("NORMAL MODE: Selected left camera tag: " + leftTagID);
          return leftTagID;
        }
      }
      
      // Fall back to right camera
      if (LimelightHelpers.getTV("limelight-right")) {
        double rightTagID = LimelightHelpers.getFiducialID("limelight-right");
        if (rightTagID > 0) {
          System.out.println("NORMAL MODE: Selected right camera tag: " + rightTagID);
          return rightTagID;
        }
      }
    }
    
    return 1; // Default tag ID if no cameras see anything
  }

  private void updateAlignmentProgress(double currentErrorDistance) {
    if (lastErrorDistance != Double.MAX_VALUE) {
      double improvement = lastErrorDistance - currentErrorDistance;
      double improvementThreshold = 0.05;
      
      if (improvement > improvementThreshold) {
        alignmentProgressTimer.restart();
        makingProgress = true;
      } else if (alignmentProgressTimer.hasElapsed(ALIGNMENT_PROGRESS_TIMEOUT)) {
        makingProgress = false;
      }
    }
    
    lastErrorDistance = currentErrorDistance;
  }

  // NEW: Get current detected tag ID for elevator position selection
  public double getCurrentTagID() {
    return tagID;
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
    
    // ENHANCED: Stop appropriate motor based on mode if it was activated
    if (elevator != null) {
      if (shooterActivated) {
        elevator.setShooterSpeed(0.0);
        System.out.println("ALIGNMENT END: Stopping shooter");
      }
      if (algaeActivated) {
        elevator.setAlgaeKickerSpeed(ElevatorConstants.ALGAE_KICKER_STOP);
        elevator.releaseShooterLock(); // NEW: Release shooter from brake mode lock
        System.out.println("ALIGNMENT END: Stopping algae kicker and releasing shooter lock");
      }
    }
    
    // Enhanced logging for algae mode
    if (isAlgaeKickingMode) {
      if (interrupted || driverOverride) {
        System.out.println("ALGAE ALIGNMENT ENDED: " + (driverOverride ? "Driver override" : "Interrupted"));
        System.out.println("SHOOTER RELEASED: Brake mode lock removed due to command end");
      } else {
        System.out.println("ALGAE ALIGNMENT COMPLETED: Successfully finished, shooter lock released");
      }
    }
  }

  @Override
  public boolean isFinished() {
    // End immediately if driver override detected
    if (driverOverride) {
      return true;
    }

    // FIXED: Check if alignment and continuous motor sequence is complete
    boolean isAligned = rotController.atSetpoint() &&
                        yController.atSetpoint() &&
                        xController.atSetpoint();

    boolean neitherCameraSeesTag = !LimelightHelpers.getTV(primaryLimelight) && !LimelightHelpers.getTV(secondaryLimelight);
    
    // For algae mode, finish when aligned and algae kicker sequence is complete
    if (isAlgaeKickingMode) {
      if (elevator != null) {
        // FIXED: Only finish after complete continuous algae kicker run
        if (alignmentCompleted && algaeActivated && shooterTimer.get() >= SHOOTER_RUN_TIME) {
          System.out.println("ALGAE ALIGNMENT COMMAND COMPLETE: Alignment + Continuous algae kicker finished");
          return true;
        }
      } else {
        // No elevator provided, finish immediately after alignment
        return isAligned && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME);
      }
    } else {
      // For normal mode with continuous shooter sequence
      if (elevator != null) {
        // FIXED: Only finish after complete continuous shooter run
        if (alignmentCompleted && shooterActivated && shooterTimer.get() >= SHOOTER_RUN_TIME) {
          System.out.println("ALIGNREEFREL: Aligned + Shot");
          return true;
        }
      } else {
        // No elevator provided, finish immediately after alignment
        return isAligned && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME);
      }
    }
    
    // FIXED: Check timeout condition for both modes
    if (neitherCameraSeesTag && dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME)) {
      return true; // Timeout - no tags visible for too long
    }
    
    // Don't finish until motor sequence is complete or timeout
    return false;
  }

  // ENHANCED: Update SmartDashboard debugging method for both motor types
  private void updateMotorDebugInfo() {
    SmartDashboard.putBoolean("Alignment Completed", alignmentCompleted);
    SmartDashboard.putBoolean("Shooter Active", shooterActivated);
    SmartDashboard.putBoolean("Algae Active", algaeActivated); // NEW: Track algae kicker state
    SmartDashboard.putBoolean("Driver Override", driverOverride);
    SmartDashboard.putBoolean("Is Algae Mode", isAlgaeKickingMode);
    
    // NEW: Enhanced motor status tracking for algae mode
    if (isAlgaeKickingMode && algaeActivated) {
      SmartDashboard.putString("Algae Alignment Status", "ACTIVE - SHOOTER LOCKED");
      SmartDashboard.putBoolean("Shooter Position Locked", true);
      SmartDashboard.putString("Shooter Lock Reason", "ALGAE ALIGNMENT");
    } else {
      SmartDashboard.putBoolean("Shooter Position Locked", false);
      SmartDashboard.putString("Shooter Lock Reason", "NONE");
    }
    
    if (shooterActivated || algaeActivated) {
      double currentTime = shooterTimer.get();
      SmartDashboard.putNumber("Motor Timer", currentTime);
      SmartDashboard.putNumber("Motor Time Remaining", SHOOTER_RUN_TIME - currentTime);
      SmartDashboard.putBoolean("Motor Running", currentTime < SHOOTER_RUN_TIME);
      SmartDashboard.putBoolean("Motor Timer Elapsed", currentTime >= SHOOTER_RUN_TIME);
      SmartDashboard.putString("Active Motor Type", isAlgaeKickingMode ? "ALGAE_KICKER" : "SHOOTER");
    } else {
      SmartDashboard.putNumber("Motor Timer", 0.0);
      SmartDashboard.putNumber("Motor Time Remaining", 0.0);
      SmartDashboard.putBoolean("Motor Running", false);
      SmartDashboard.putBoolean("Motor Timer Elapsed", false);
      SmartDashboard.putString("Active Motor Type", "NONE");
    }
    
    // Additional debug info
    boolean isCurrentlyAligned = rotController.atSetpoint() &&
                                yController.atSetpoint() &&
                                xController.atSetpoint();
    SmartDashboard.putBoolean("Currently At Setpoint", isCurrentlyAligned);
    SmartDashboard.putNumber("Validation Timer", stopTimer.get());
    SmartDashboard.putBoolean("Validation Time Met", stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME));
    
    // Enhanced phase tracking with specific motor mode indication
    String phase = "ALIGNING";
    if (driverOverride) {
      phase = isAlgaeKickingMode ? "ALGAE_CANCELLED" : "CANCELLED";
    } else if (alignmentCompleted) {
      if (shooterActivated || algaeActivated) {
        if (shooterTimer.get() < SHOOTER_RUN_TIME) {
          phase = isAlgaeKickingMode ? "ALGAE_KICKING_CONTINUOUS" : "SHOOTING_CONTINUOUS";
        } else {
          phase = isAlgaeKickingMode ? "ALGAE_KICK_COMPLETE" : "SHOOT_COMPLETE";
        }
      } else {
        phase = isAlgaeKickingMode ? "ALGAE_READY" : "READY_TO_SHOOT";
      }
    } else {
      phase = isAlgaeKickingMode ? "ALGAE_ALIGNING" : "ALIGNING";
    }
    SmartDashboard.putString("Alignment Phase", phase);
  }
}