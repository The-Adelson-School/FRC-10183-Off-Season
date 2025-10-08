package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

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

  // NEW: Constructor with algae kicking mode
  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase, boolean forceLeftCameraPriority, boolean isAlgaeKickingMode) {
    xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);
    yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
    rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0.0);
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.isAlgaeKickingMode = isAlgaeKickingMode;
    this.switchDelayTimer = new Timer();
    this.alignmentProgressTimer = new Timer();
    
    // SPECIAL CASE: Force left camera priority for algae kicking (X button)
    if (forceLeftCameraPriority) {
      primaryLimelight = "limelight-left";   // Always prioritize left camera
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

  // Keep existing constructors for backward compatibility
  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    this(isRightScore, drivebase, false, false);
  }

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase, boolean forceLeftCameraPriority) {
    this(isRightScore, drivebase, forceLeftCameraPriority, false);
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
    
    System.out.println("Smart Dual Limelight alignment started - Alignment Side: " + (isRightScore ? "RIGHT" : "LEFT") + 
                      ", Primary Camera: " + primaryLimelight + ", Secondary Camera: " + secondaryLimelight + 
                      ", Starting Camera: " + currentLimelightName);
  }

  @Override
  public void execute() {
    updateCurrentLimelightSmart();
    
    double detectedTagID = LimelightHelpers.getFiducialID(currentLimelightName);
    
    if (LimelightHelpers.getTV(currentLimelightName) && detectedTagID == tagID) {
      this.dontSeeTagTimer.reset();
      
      double[] positions = LimelightHelpers.getBotPose_TargetSpace(currentLimelightName);
      SmartDashboard.putNumber("x", positions[2]);
      SmartDashboard.putString("Active Camera", currentLimelightName);
      SmartDashboard.putBoolean("Primary Camera Active", currentLimelightName.equals(primaryLimelight));

      // ENHANCED DEBUGGING for Y position and control
      SmartDashboard.putNumber("Raw Y Position", positions[0]);
      SmartDashboard.putNumber("Y Setpoint", yController.getSetpoint());
      SmartDashboard.putNumber("Y Error", positions[0] - yController.getSetpoint());

      double xSpeed = xController.calculate(positions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(positions[0]);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("Raw Y Speed (before negation)", yController.calculate(positions[0]));
      double rotValue = rotController.calculate(positions[4]);

      double currentErrorDistance = Math.sqrt(
          Math.pow(positions[2], 2) + 
          Math.pow(positions[0], 2) + 
          Math.pow(positions[4], 2)
      );
      
      updateAlignmentProgress(currentErrorDistance);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
      SmartDashboard.putString("Active Camera", currentLimelightName + " (NO TARGET)");
      makingProgress = false;
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    SmartDashboard.putString("Camera Status", getCameraStatus());
    SmartDashboard.putBoolean("Making Alignment Progress", makingProgress);
    SmartDashboard.putNumber("Current Error Distance", lastErrorDistance);
  }

  private void updateCurrentLimelightSmart() {
    boolean primaryHasTarget = LimelightHelpers.getTV(primaryLimelight) && 
                              LimelightHelpers.getFiducialID(primaryLimelight) == tagID;
    boolean secondaryHasTarget = LimelightHelpers.getTV(secondaryLimelight) && 
                                LimelightHelpers.getFiducialID(secondaryLimelight) == tagID;
    
    boolean shouldSwitch = false;
    String targetCamera = currentLimelightName;
    String switchReason = "";
    
    // REMOVED: Distance-based switching logic entirely
    // This was causing the system to switch away from the alignment-side camera
    // Now we ONLY use alignment side preference and target availability
    
    // STRICT ALIGNMENT SIDE PREFERENCE: Only switch based on target availability and progress
    if (currentLimelightName.equals(primaryLimelight)) {
      // Currently using primary (preferred) camera
      // ONLY switch to secondary if primary loses target AND secondary has target
      if (!primaryHasTarget && secondaryHasTarget) {
        shouldSwitch = true;
        targetCamera = secondaryLimelight;
        switchReason = "Primary camera lost target, secondary has target";
      } else if (!makingProgress && secondaryHasTarget && alignmentProgressTimer.hasElapsed(ALIGNMENT_PROGRESS_TIMEOUT)) {
        shouldSwitch = true;
        targetCamera = secondaryLimelight;
        switchReason = "Primary camera not making progress, trying secondary";
      }
      // NO OTHER SWITCHING CONDITIONS - primary camera is strongly preferred
    } else {
      // Currently using secondary (fallback) camera
      // IMMEDIATELY switch back to primary if it has target (prioritize primary)
      if (primaryHasTarget) {
        shouldSwitch = true;
        targetCamera = primaryLimelight;
        switchReason = "Primary camera regained target, switching back to preferred camera";
      } else if (!secondaryHasTarget && primaryHasTarget) {
        shouldSwitch = true;
        targetCamera = primaryLimelight;
        switchReason = "Secondary lost target, primary has target";
      }
    }
    
    if (shouldSwitch && switchDelayTimer.hasElapsed(SWITCH_DELAY)) {
      currentLimelightName = targetCamera;
      updateControllerSetpoints();
      switchDelayTimer.restart();
      alignmentProgressTimer.restart();
      lastErrorDistance = Double.MAX_VALUE;
      makingProgress = true;
      System.out.println("CAMERA SWITCH: " + switchReason + " -> " + targetCamera + " (Tag ID: " + tagID + ", setpoints updated)");
    }
    
    SmartDashboard.putBoolean("Primary Has Target", primaryHasTarget);
    SmartDashboard.putBoolean("Secondary Has Target", secondaryHasTarget);
    SmartDashboard.putNumber("Switch Timer", switchDelayTimer.get());
    SmartDashboard.putNumber("Progress Timer", alignmentProgressTimer.get());
    SmartDashboard.putString("Switch Reason", switchReason);
    SmartDashboard.putString("Current Camera Setpoints", getCurrentSetpointsString());
    
    // ENHANCED DEBUGGING for strict alignment side preference
    SmartDashboard.putString("Alignment Side", isRightScore ? "RIGHT" : "LEFT");
    SmartDashboard.putString("Primary Camera (Preferred)", primaryLimelight);
    SmartDashboard.putString("Secondary Camera (Fallback)", secondaryLimelight);
    SmartDashboard.putString("Current Camera", currentLimelightName);
    SmartDashboard.putBoolean("Using Preferred Camera", currentLimelightName.equals(primaryLimelight));
    SmartDashboard.putNumber("Current Target Tag ID", tagID);
    SmartDashboard.putString("Camera Switching Logic", "STRICT ALIGNMENT SIDE PREFERENCE - No distance switching");
    
    // Show when multiple tags are available but we're maintaining side preference
    boolean multipleTagsAvailable = LimelightHelpers.getTV(primaryLimelight) && LimelightHelpers.getTV(secondaryLimelight) &&
        LimelightHelpers.getFiducialID(primaryLimelight) != LimelightHelpers.getFiducialID(secondaryLimelight);
    SmartDashboard.putBoolean("Multiple Tags Available", multipleTagsAvailable);
    if (multipleTagsAvailable) {
      SmartDashboard.putString("Multiple Tags Strategy", "Using alignment side preference - ignoring distance");
    }
  }

  private void updateControllerSetpoints() {
    if (currentLimelightName.equals("limelight-right")) {
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_RIGHT);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT_RIGHT);
      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT_RIGHT);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT_RIGHT);
      
      if (isRightScore) {
        yController.setSetpoint(Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT));
      } else {
        yController.setSetpoint(-Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT));
      }
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT_RIGHT);
    } else {
      // LEFT CAMERA - Different logic needed with special cross-camera offset
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_LEFT);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT_LEFT);
      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT_LEFT);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT_LEFT);
      
      // ALGAE KICKING MODE: Apply algae-specific offset
      if (isAlgaeKickingMode) {
        // Center alignment with algae-specific offset
        double algaeOffset = VisionConstants.Y_OFFSET_ALGAE_KICKER;
        yController.setSetpoint(algaeOffset);
        
        SmartDashboard.putString("Special Offset Applied", "ALGAE KICKING MODE");
        SmartDashboard.putNumber("Algae Y Offset", algaeOffset);
        SmartDashboard.putNumber("Final Y Setpoint", algaeOffset);
      } else if (isRightScore) {
        // RIGHT side alignment using left camera - use negative setpoint PLUS special offset
        double baseYSetpoint = -Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
        double specialOffset = VisionConstants.Y_OFFSET_RIGHT_ALIGN_ON_LEFT_CAMERA;
        yController.setSetpoint(baseYSetpoint + specialOffset);
        
        SmartDashboard.putString("Special Offset Applied", "RIGHT alignment on LEFT camera");
        SmartDashboard.putNumber("Base Y Setpoint", baseYSetpoint);
        SmartDashboard.putNumber("Special Offset", specialOffset);
        SmartDashboard.putNumber("Final Y Setpoint", baseYSetpoint + specialOffset);
      } else {
        // LEFT side alignment using left camera - use positive setpoint (no special offset needed)
        yController.setSetpoint(Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT));
        SmartDashboard.putString("Special Offset Applied", "None - LEFT on LEFT camera");
      }
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT_LEFT);
    }
    
    SmartDashboard.putString("Active Camera", currentLimelightName);
    SmartDashboard.putString("Active Setpoints", getCurrentSetpointsString());
    SmartDashboard.putBoolean("Is Right Score", isRightScore);
    SmartDashboard.putString("Alignment Side", isRightScore ? "RIGHT" : "LEFT");
    SmartDashboard.putBoolean("Algae Kicking Mode", isAlgaeKickingMode);
    
    // ENHANCED DEBUGGING for special offset scenario
    SmartDashboard.putNumber("Y Setpoint Used", yController.getSetpoint());
    String cameraLogic = currentLimelightName.equals("limelight-right") ? 
        (isRightScore ? "+RIGHT" : "-LEFT") : 
        (isAlgaeKickingMode ? "ALGAE_CENTER+OFFSET" :
         (isRightScore ? "-RIGHT_ON_LEFT_CAM+OFFSET" : "+LEFT_ON_LEFT_CAM"));
    SmartDashboard.putString("Camera Y Logic", cameraLogic);
    SmartDashboard.putBoolean("Using Cross-Camera Offset", 
        currentLimelightName.equals("limelight-left") && isRightScore && !isAlgaeKickingMode);
    SmartDashboard.putBoolean("Using Algae Offset", 
        currentLimelightName.equals("limelight-left") && isAlgaeKickingMode);
  }

  private String getCurrentSetpointsString() {
    return String.format("X:%.2f Y:%.2f R:%.2f (%s)", 
                        xController.getSetpoint(),
                        yController.getSetpoint(), 
                        rotController.getSetpoint(),
                        currentLimelightName.equals("limelight-right") ? "RIGHT" : "LEFT");
  }

  private boolean isAlignmentBetter(String cameraToCheck) {
    if (!LimelightHelpers.getTV(cameraToCheck)) {
      return false;
    }
    
    double[] positions = LimelightHelpers.getBotPose_TargetSpace(cameraToCheck);
    if (positions.length < 6) {
      return false;
    }
    
    double xSetpoint, ySetpoint, rotSetpoint;
    if (cameraToCheck.equals("limelight-right")) {
      xSetpoint = VisionConstants.X_SETPOINT_REEF_ALIGNMENT_RIGHT;
      ySetpoint = isRightScore ? Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT) : -Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT);
      rotSetpoint = VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_RIGHT;
    } else {
      xSetpoint = VisionConstants.X_SETPOINT_REEF_ALIGNMENT_LEFT;
      
      // APPLY SAME SPECIAL OFFSET LOGIC FOR COMPARISON
      if (isRightScore) {
        // RIGHT alignment on LEFT camera - include special offset
        double baseYSetpoint = -Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
        ySetpoint = baseYSetpoint + VisionConstants.Y_OFFSET_RIGHT_ALIGN_ON_LEFT_CAMERA;
      } else {
        // LEFT alignment on LEFT camera - no special offset
        ySetpoint = Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
      }
      
      rotSetpoint = VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_LEFT;
    }
    
    double errorDistance = Math.sqrt(
        Math.pow(positions[2] - xSetpoint, 2) + 
        Math.pow(positions[0] - ySetpoint, 2) + 
        Math.pow(positions[4] - rotSetpoint, 2)
    );
    
    return errorDistance < (lastErrorDistance * 0.8);
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
    SmartDashboard.putNumber("Alignment Improvement", lastErrorDistance - currentErrorDistance);
  }

  /**
   * Get the best initial camera - prefer alignment-side-specific primary camera
   */
  private String getBestInitialCamera() {
    boolean primaryHasTarget = LimelightHelpers.getTV(primaryLimelight);
    boolean secondaryHasTarget = LimelightHelpers.getTV(secondaryLimelight);
    
    // ALWAYS prefer the primary camera for this alignment side if it has target
    if (primaryHasTarget) {
      return primaryLimelight;
    } else if (secondaryHasTarget) {
      return secondaryLimelight;
    } else {
      return primaryLimelight; // Default to primary even if no target
    }
  }

  /**
   * Get the best available tag ID from either camera - now prioritizes alignment side over distance
   */
  private double getBestTagID() {
    // FIXED: Prioritize alignment side preference over distance
    // Always try the primary camera first (based on alignment side)
    if (LimelightHelpers.getTV(primaryLimelight)) {
      double primaryTagID = LimelightHelpers.getFiducialID(primaryLimelight);
      if (primaryTagID > 0) {
        System.out.println("Selected primary camera tag: " + primaryTagID + " (alignment side preference)");
        return primaryTagID;
      }
    }
    
    // Only fall back to secondary camera if primary has no target
    if (LimelightHelpers.getTV(secondaryLimelight)) {
      double secondaryTagID = LimelightHelpers.getFiducialID(secondaryLimelight);
      if (secondaryTagID > 0) {
        System.out.println("Selected secondary camera tag: " + secondaryTagID + " (primary camera unavailable)");
        return secondaryTagID;
      }
    }
    
    // If both cameras see different tags, prioritize the primary camera's tag
    // This maintains alignment side consistency
    if (LimelightHelpers.getTV(primaryLimelight) && LimelightHelpers.getTV(secondaryLimelight)) {
      double primaryTagID = LimelightHelpers.getFiducialID(primaryLimelight);
      double secondaryTagID = LimelightHelpers.getFiducialID(secondaryLimelight);
      
      if (primaryTagID != secondaryTagID && primaryTagID > 0 && secondaryTagID > 0) {
        System.out.println("Multiple different tags detected - using primary camera tag: " + primaryTagID + " (maintaining " + (isRightScore ? "RIGHT" : "LEFT") + " alignment side)");
        return primaryTagID; // Always prefer primary camera's tag for alignment consistency
      }
    }
    
    return 1; // Default tag ID if no cameras see anything
  }

  private String getCameraStatus() {
    boolean primaryTV = LimelightHelpers.getTV(primaryLimelight);
    boolean secondaryTV = LimelightHelpers.getTV(secondaryLimelight);
    double primaryTagID = LimelightHelpers.getFiducialID(primaryLimelight);
    double secondaryTagID = LimelightHelpers.getFiducialID(secondaryLimelight);
    
    String primaryLabel = primaryLimelight.equals("limelight-right") ? "R" : "L";
    String secondaryLabel = secondaryLimelight.equals("limelight-right") ? "R" : "L";
    String currentLabel = currentLimelightName.equals(primaryLimelight) ? "PRIMARY" : "SECONDARY";
    
    return String.format("P(%s):%s(ID:%.0f) S(%s):%s(ID:%.0f) Current:%s", 
                        primaryLabel, primaryTV ? "✓" : "✗", primaryTagID,
                        secondaryLabel, secondaryTV ? "✓" : "✗", secondaryTagID,
                        currentLabel);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    boolean isAligned = rotController.atSetpoint() &&
                        yController.atSetpoint() &&
                        xController.atSetpoint();

    SmartDashboard.putBoolean("Is Aligned", isAligned);
    SmartDashboard.putNumber("Stop Timer", stopTimer.get());
    SmartDashboard.putBoolean("Dont See Tag Timer Elapsed", dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME));

    boolean neitherCameraSeesTag = !LimelightHelpers.getTV(primaryLimelight) && !LimelightHelpers.getTV(secondaryLimelight);
    
    return (isAligned && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME)) ||
           (neitherCameraSeesTag && dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME));
  }

  // NEW: Get current detected tag ID for elevator position selection
  public double getCurrentTagID() {
    return tagID;
  }
}