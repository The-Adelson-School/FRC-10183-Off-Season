package frc.robot.subsystems.swervedrive;

import java.util.function.Consumer;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimeLightStuff {
    
    /**
     * Represents a configured Limelight camera
     */
    public static class LimelightCamera {
        private final String name;
        
        public LimelightCamera(String name) {
            this.name = name;
        }
        
        public String getName() { return name; }
    }
    
    private final LimelightCamera rightCamera;
    private final LimelightCamera leftCamera;
    private final Consumer<VisionMeasurement> poseConsumer;
    
    // Shuffleboard tabs for organized display
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    private final ShuffleboardTab debugTab = Shuffleboard.getTab("Vision Debug");
    
    // Pre-create dashboard entries to avoid duplicate creation
    private final GenericEntry rightCameraMeasurementsEntry = visionTab.add("Right Camera Measurements", 0).getEntry();
    private final GenericEntry leftCameraMeasurementsEntry = visionTab.add("Left Camera Measurements", 0).getEntry();
    private final GenericEntry rejectedMeasurementsEntry = visionTab.add("Rejected Measurements", 0).getEntry();
    private final GenericEntry totalMeasurementsEntry = visionTab.add("Total Measurements", 0).getEntry();
    private final GenericEntry acceptanceRateEntry = visionTab.add("Acceptance Rate (%)", 0.0).getEntry();
    
    private final GenericEntry rightCameraHasTargetEntry = visionTab.add("Right Camera Has Target", false).getEntry();
    private final GenericEntry leftCameraHasTargetEntry = visionTab.add("Left Camera Has Target", false).getEntry();
    private final GenericEntry rightCameraTagCountEntry = visionTab.add("Right Camera Tag Count", 0).getEntry();
    private final GenericEntry leftCameraTagCountEntry = visionTab.add("Left Camera Tag Count", 0).getEntry();
    
    // MegaTag pose entries
    private final GenericEntry rightCameraMegaTagXEntry = visionTab.add("Right MegaTag X", 0.0).getEntry();
    private final GenericEntry rightCameraMegaTagYEntry = visionTab.add("Right MegaTag Y", 0.0).getEntry();
    private final GenericEntry rightCameraMegaTagRotationEntry = visionTab.add("Right MegaTag Rotation", 0.0).getEntry();
    private final GenericEntry leftCameraMegaTagXEntry = visionTab.add("Left MegaTag X", 0.0).getEntry();
    private final GenericEntry leftCameraMegaTagYEntry = visionTab.add("Left MegaTag Y", 0.0).getEntry();
    private final GenericEntry leftCameraMegaTagRotationEntry = visionTab.add("Left MegaTag Rotation", 0.0).getEntry();
    
    private final GenericEntry lastMeasurementSourceEntry = debugTab.add("Last Measurement Source", "").getEntry();
    private final GenericEntry lastMeasurementConfidenceEntry = debugTab.add("Last Measurement Confidence", 0.0).getEntry();
    private final GenericEntry lastMeasurementTagsEntry = debugTab.add("Last Measurement Tags", 0).getEntry();
    private final GenericEntry lastMeasurementDistanceEntry = debugTab.add("Last Measurement Distance", 0.0).getEntry();
    private final GenericEntry lastRejectedReasonEntry = debugTab.add("Last Rejected Reason", "").getEntry();
    
    // Statistics tracking
    private int rightCameraMeasurements = 0;
    private int leftCameraMeasurements = 0;
    private int rejectedMeasurements = 0;
    
    // Track last processed timestamps to filter duplicate poses
    private double lastRightCameraTimestamp = -1.0;
    private double lastLeftCameraTimestamp = -1.0;
    
    // Store the single current pose for each camera (eliminates ghost robots)
    private Pose2d currentRightCameraPose = null;
    private Pose2d currentLeftCameraPose = null;
    
    // One-time odometry reset tracking
    private boolean hasPerformedInitialOdometryReset = false;
    private final Consumer<Pose2d> odometryResetCallback;
    
    /**
     * Represents a vision measurement with pose, timestamp, and confidence data
     */
    public static class VisionMeasurement {
        public final Pose2d pose;
        public final double timestampSeconds;
        public final double confidence;
        public final String source;
        public final int tagCount;
        
        public VisionMeasurement(Pose2d pose, double timestampSeconds, double confidence, 
                               String source, int tagCount) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.confidence = confidence;
            this.source = source;
            this.tagCount = tagCount;
        }
    }
    
    /**
     * Creates a new LimeLightStuff instance with configured cameras
     * 
     * @param poseConsumer Function to consume vision measurements (typically SwerveDrive::addVisionMeasurement)
     */
    public LimeLightStuff(Consumer<VisionMeasurement> poseConsumer) {
        this(poseConsumer, null);
    }
    
    /**
     * Creates a new LimeLightStuff instance with configured cameras and odometry reset
     * 
     * @param poseConsumer Function to consume vision measurements
     * @param odometryResetCallback Function to reset odometry (optional)
     */
    public LimeLightStuff(Consumer<VisionMeasurement> poseConsumer, Consumer<Pose2d> odometryResetCallback) {
        this.poseConsumer = poseConsumer;
        this.odometryResetCallback = odometryResetCallback;
        
        // Configure cameras with simple names - transforms handled in Limelight interface
        this.rightCamera = new LimelightCamera(VisionConstants.RIGHT_LIMELIGHT_NAME);
        this.leftCamera = new LimelightCamera(VisionConstants.LEFT_LIMELIGHT_NAME);
        
        // Enable MegaTag2 mode for better accuracy (if supported)
        configureLimelight(rightCamera.getName());
        configureLimelight(leftCamera.getName());
    }
    
    /**
     * Configure Limelight settings for optimal AprilTag detection
     */
    private void configureLimelight(String limelightName) {
        // Set to AprilTag pipeline (adjust pipeline number as needed)
        LimelightHelpers.setPipelineIndex(limelightName, VisionConstants.APRILTAG_PIPELINE_INDEX);
        
        // Configure LED mode
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        
        // Set stream mode for debugging
        LimelightHelpers.setStreamMode_Standard(limelightName);
        
        // Allow all AprilTags - no ID filtering
        // Both Limelights can now see and use any AprilTag for localization
        // Transform configuration is handled in the Limelight interface, not here
    }
    
    /**
     * Updates robot orientation for MegaTag2 (call this every loop with current robot state)
     * NOTE: This ONLY sends robot orientation TO the Limelight for better pose estimation.
     * MegaTag does NOT send gyro measurements back to odometry - only vision pose estimates.
     * 
     * @param yawRadians Robot yaw angle in radians
     * @param yawRateRadPerSec Robot angular velocity in radians per second
     */
    public void updateRobotOrientation(double yawRadians, double yawRateRadPerSec) {
        if (VisionConstants.ENABLE_MEGATAG2) {
            // Convert radians to degrees for Limelight (Limelight expects degrees)
            double yawDegrees = Math.toDegrees(yawRadians);
            double yawRateDegPerSec = Math.toDegrees(yawRateRadPerSec);
            
            // Debug output
        
            
            // IMPORTANT: This sends robot orientation TO the Limelight to help with pose estimation
            // The Limelight uses this data internally but does NOT send gyro data back to our odometry
            // Only vision-based pose estimates are sent back via getBotPoseEstimate_wpi*()
            LimelightHelpers.SetRobotOrientation(
                rightCamera.getName(), 
                yawDegrees,          // Convert to degrees for Limelight
                yawRateDegPerSec,    // Convert to degrees per second for Limelight
                0.0,   // pitch
                0.0,   // pitch rate
                0.0,   // roll
                0.0    // roll rate
            );
            LimelightHelpers.SetRobotOrientation(
                leftCamera.getName(), 
                yawDegrees,          // Convert to degrees for Limelight
                yawRateDegPerSec,    // Convert to degrees per second for Limelight
                0.0,   // pitch
                0.0,   // pitch rate
                0.0,   // roll
                0.0    // roll rate
            );
            
        } else {
        }
    }
    
    /**
     * Overload for backward compatibility - accepts degrees and converts to radians
     * @deprecated Use updateRobotOrientation(double yawRadians, double yawRateRadPerSec) instead
     */
    @Deprecated
    public void updateRobotOrientation(double yawDegrees) {
        updateRobotOrientation(Math.toRadians(yawDegrees), 0.0);
    }

    public void processVisionMeasurements() {
        // Process each camera
        //auto brandon riley todo
        processCameraMeasurements(rightCamera);
        processCameraMeasurements(leftCamera);
        
        // Update dashboard statistics
        //updateDashboard();
    }
    
    /**
     * Process measurements from a specific camera
     */
    private void processCameraMeasurements(LimelightCamera camera) {
        try {
            // Get alliance-aware pose estimate
            // IMPORTANT: This gets vision-based pose estimates from MegaTag, NOT gyro data
            // MegaTag uses our robot orientation (sent above) to improve its pose calculation,
            // but only returns vision-derived poses, not IMU/gyro measurements
            PoseEstimate poseEstimate = getAllianceAwarePoseEstimate(camera.getName());
            
            if (poseEstimate == null || poseEstimate.pose == null) {
                return; // No pose data available
            }
            
            // CRITICAL: Filter out duplicate/historical poses to prevent ghost robots
            boolean isRightCamera = (camera == rightCamera);
            double lastTimestamp = isRightCamera ? lastRightCameraTimestamp : lastLeftCameraTimestamp;
            
            // Only process if this is a newer timestamp (prevents processing pose history/duplicates)
            if (poseEstimate.timestampSeconds <= lastTimestamp) {
                return; // Skip older/duplicate poses
            }
            
            // Update timestamp tracking
            if (isRightCamera) {
                lastRightCameraTimestamp = poseEstimate.timestampSeconds;
                currentRightCameraPose = poseEstimate.pose; // Store single current pose
            } else {
                lastLeftCameraTimestamp = poseEstimate.timestampSeconds;
                currentLeftCameraPose = poseEstimate.pose; // Store single current pose
            }
            
            // Update MegaTag pose on dashboard - ONLY the single current pose (no ghosts)
            if (isRightCamera) {
                rightCameraMegaTagXEntry.setDouble(poseEstimate.pose.getX());
                rightCameraMegaTagYEntry.setDouble(poseEstimate.pose.getY());
                rightCameraMegaTagRotationEntry.setDouble(poseEstimate.pose.getRotation().getDegrees());
                
                // Also put on main SmartDashboard for easy access - SINGLE POSE ONLY
             /*   SmartDashboard.putNumber("Right MegaTag X", poseEstimate.pose.getX());
                SmartDashboard.putNumber("Right MegaTag Y", poseEstimate.pose.getY());
                SmartDashboard.putNumber("Right MegaTag Rotation", poseEstimate.pose.getRotation().getDegrees());
                SmartDashboard.putString("Right MegaTag Pose", String.format("(%.2f, %.2f, %.1f°)", 
                    poseEstimate.pose.getX(), poseEstimate.pose.getY(), poseEstimate.pose.getRotation().getDegrees()));
                SmartDashboard.putNumber("Right Camera Last Timestamp", poseEstimate.timestampSeconds);
           */
                } else {
                leftCameraMegaTagXEntry.setDouble(poseEstimate.pose.getX());
                leftCameraMegaTagYEntry.setDouble(poseEstimate.pose.getY());
                leftCameraMegaTagRotationEntry.setDouble(poseEstimate.pose.getRotation().getDegrees());
                
              /*  // Also put on main SmartDashboard for easy access - SINGLE POSE ONLY
                SmartDashboard.putNumber("Left MegaTag X", poseEstimate.pose.getX());
                SmartDashboard.putNumber("Left MegaTag Y", poseEstimate.pose.getY());
                SmartDashboard.putNumber("Left MegaTag Rotation", poseEstimate.pose.getRotation().getDegrees());
                SmartDashboard.putString("Left MegaTag Pose", String.format("(%.2f, %.2f, %.1f°)", 
                    poseEstimate.pose.getX(), poseEstimate.pose.getY(), poseEstimate.pose.getRotation().getDegrees()));
                SmartDashboard.putNumber("Left Camera Last Timestamp", poseEstimate.timestampSeconds);
            */
                }
            
            // Combined MegaTag info - shows which camera has the most recent valid reading
           /* SmartDashboard.putString("Latest MegaTag Source", camera.getName());
            SmartDashboard.putString("Latest MegaTag Pose", String.format("(%.2f, %.2f, %.1f°)", 
                poseEstimate.pose.getX(), poseEstimate.pose.getY(), poseEstimate.pose.getRotation().getDegrees()));
            SmartDashboard.putNumber("Latest MegaTag Timestamp", poseEstimate.timestampSeconds);
            SmartDashboard.putString("Alliance", isRedAlliance() ? "RED" : "BLUE");
            */
            if (!LimelightHelpers.validPoseEstimate(poseEstimate)) {
                return; // No valid measurement for odometry
            }
            
            // Validate the measurement for odometry use
            if (isValidMeasurement(poseEstimate, camera)) {
                // ONE-TIME ODOMETRY RESET: Reset odometry on first trusted vision measurement
                if (!hasPerformedInitialOdometryReset && odometryResetCallback != null) {
                    hasPerformedInitialOdometryReset = true;
                    odometryResetCallback.accept(poseEstimate.pose);
    
                }
                
                // Calculate confidence based on tag count, distance, and ambiguity
                double confidence = calculateConfidence(poseEstimate);
                
                VisionMeasurement measurement = new VisionMeasurement(
                    poseEstimate.pose,
                    poseEstimate.timestampSeconds,
                    confidence,
                    camera.getName(),
                    poseEstimate.tagCount
                );
                
                // Send measurement to odometry - THIS IS THE KEY INTEGRATION POINT
                // NOTE: This sends VISION-BASED pose estimates to odometry, NOT gyro data
                // The pose comes from AprilTag detection enhanced by MegaTag processing
                poseConsumer.accept(measurement);
                
                // Update statistics
                if (isRightCamera) {
                    rightCameraMeasurements++;
                } else {
                    leftCameraMeasurements++;
                }
                
                // Debug output - shows active vision integration
                lastMeasurementSourceEntry.setString(camera.getName());
                lastMeasurementConfidenceEntry.setDouble(confidence);
                lastMeasurementTagsEntry.setDouble(poseEstimate.tagCount);
                lastMeasurementDistanceEntry.setDouble(poseEstimate.avgTagDist);
                
                // Log successful measurement processing (reduced frequency to avoid spam)
                if ((System.currentTimeMillis() % 1000) < 50) { // Only log every ~1 second
                    System.out.println(String.format("Vision Update: %s (%s) - Tags: %d, Confidence: %.3f, Distance: %.3fm, TS: %.3f", 
                        camera.getName(), isRedAlliance() ? "RED" : "BLUE", poseEstimate.tagCount, confidence, poseEstimate.avgTagDist, poseEstimate.timestampSeconds));
                }
                    
            } else {
                rejectedMeasurements++;
                // Debug rejected measurements
                lastRejectedReasonEntry.setString(getRejectionReason(poseEstimate, camera));
            }
            
        } catch (Exception e) {
            DriverStation.reportError("Vision processing error for " + camera.getName() + ": " + e.getMessage(), false);
        }
    }
    
    /**
     * Get alliance-aware pose estimate from Limelight
     */
    private PoseEstimate getAllianceAwarePoseEstimate(String cameraName) {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        
    }
    
    /**
     * Public method to check if we're on red alliance (for other classes)
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Validate if a pose estimate should be accepted
     */
    private boolean isValidMeasurement(PoseEstimate poseEstimate, LimelightCamera camera) {
        // Check if we have valid fiducials
        if (poseEstimate.rawFiducials == null || poseEstimate.rawFiducials.length == 0) {
            return false;
        }
        
        // FIXED: Check Z-axis error (height) - use pose.getZ() instead of pose.getY()
        // For 2D pose, check if the pose seems reasonable (not floating)
        // Note: Pose2d doesn't have Z, but the 3D pose estimate might indicate unreasonable height
        // We'll validate the pose is reasonable by checking if it's within field bounds
        double x = poseEstimate.pose.getX();
        double y = poseEstimate.pose.getY();
        
        // Basic field bounds check (adjust for your field size)
        if (x < -1.0 || x > 17.0 || y < -1.0 || y > 9.0) {
            return false; // Outside reasonable field bounds
        }
        
        // Check individual tag ambiguity
        for (var fiducial : poseEstimate.rawFiducials) {
            if (fiducial.ambiguity > VisionConstants.MAX_AMBIGUITY) {
                return false; // Reject if any tag has high ambiguity
            }
        }
        
        // Check tag count and distance requirements
        if (poseEstimate.tagCount >= VisionConstants.MIN_TAG_COUNT) {
            // Multiple tags - allow greater distance
            return poseEstimate.avgTagDist <= VisionConstants.MAX_TAG_DISTANCE;
        } else if (poseEstimate.tagCount == 1) {
            // Single tag - stricter distance requirement and check legacy ambiguity
            boolean withinDistance = poseEstimate.avgTagDist <= VisionConstants.SINGLE_TAG_MAX_DISTANCE;
            boolean lowAmbiguity = true; // Default to true if no ambiguity data
            
            // Check legacy pose ambiguity if available (for backwards compatibility)
            if (poseEstimate.rawFiducials.length > 0) {
                lowAmbiguity = poseEstimate.rawFiducials[0].ambiguity <= VisionConstants.MAX_POSE_AMBIGUITY;
            }
            
            return withinDistance && lowAmbiguity;
        }
        
        return false; // No tags detected
    }
    
    /**
     * Get reason why a measurement was rejected (for debugging)
     */
    private String getRejectionReason(PoseEstimate poseEstimate, LimelightCamera camera) {
        if (poseEstimate.rawFiducials == null || poseEstimate.rawFiducials.length == 0) {
            return "No fiducials";
        }
        if (Math.abs(poseEstimate.pose.getY()) > VisionConstants.MAX_Z_ERROR) {
            return "Z-error too high";
        }
        for (var fiducial : poseEstimate.rawFiducials) {
            if (fiducial.ambiguity > VisionConstants.MAX_AMBIGUITY) {
                return "High ambiguity";
            }
        }
        if (poseEstimate.tagCount >= VisionConstants.MIN_TAG_COUNT) {
            if (poseEstimate.avgTagDist > VisionConstants.MAX_TAG_DISTANCE) {
                return "Distance too far (multi-tag)";
            }
        } else if (poseEstimate.tagCount == 1) {
            if (poseEstimate.avgTagDist > VisionConstants.SINGLE_TAG_MAX_DISTANCE) {
                return "Distance too far (single-tag)";
            }
        }
        return "Unknown";
    }

    /**
     * Calculate confidence/standard deviation for the measurement
     * Lower values indicate higher confidence
     */
    private double calculateConfidence(PoseEstimate poseEstimate) {
        double baseConfidence;
        
        // Adjust based on tag count (more tags = higher confidence)
        if (poseEstimate.tagCount >= 3) {
            baseConfidence = VisionConstants.BASE_CONFIDENCE_MULTI_TAGS;
        } else if (poseEstimate.tagCount == 2) {
            baseConfidence = VisionConstants.BASE_CONFIDENCE_TWO_TAGS;
        } else {
            baseConfidence = VisionConstants.BASE_CONFIDENCE_SINGLE_TAG; // Single tag is less reliable
        }
        
        // Adjust based on distance (closer tags = higher confidence)
        double distanceFactor = Math.min(poseEstimate.avgTagDist / VisionConstants.DISTANCE_SCALE, 1.0);
        baseConfidence += distanceFactor * VisionConstants.DISTANCE_WEIGHT;
        
        // Adjust based on tag area (larger tags = higher confidence)
        if (poseEstimate.avgTagArea > 0) {
            double areaFactor = Math.max(0, 1.0 - (poseEstimate.avgTagArea / VisionConstants.AREA_SCALE));
            baseConfidence += areaFactor * VisionConstants.AREA_WEIGHT;
        }
        
        return Math.max(VisionConstants.MIN_CONFIDENCE, baseConfidence);
    }
    
    /**
     * Update Shuffleboard with vision statistics and status
     */
    private void updateDashboard() {
        rightCameraMeasurementsEntry.setDouble(rightCameraMeasurements);
        leftCameraMeasurementsEntry.setDouble(leftCameraMeasurements);
        rejectedMeasurementsEntry.setDouble(rejectedMeasurements);
        totalMeasurementsEntry.setDouble(rightCameraMeasurements + leftCameraMeasurements);
        
        double acceptanceRate = (rightCameraMeasurements + leftCameraMeasurements) / 
            Math.max(1.0, rightCameraMeasurements + leftCameraMeasurements + rejectedMeasurements) * 100.0;
        acceptanceRateEntry.setDouble(acceptanceRate);
        
        // Current target information for each camera
        rightCameraHasTargetEntry.setBoolean(LimelightHelpers.getTV(rightCamera.getName()));
        leftCameraHasTargetEntry.setBoolean(LimelightHelpers.getTV(leftCamera.getName()));
        rightCameraTagCountEntry.setDouble(LimelightHelpers.getTargetCount(rightCamera.getName()));
        leftCameraTagCountEntry.setDouble(LimelightHelpers.getTargetCount(leftCamera.getName()));
        
        // MegaTag status on main SmartDashboard
        SmartDashboard.putBoolean("Right Camera Active", LimelightHelpers.getTV(rightCamera.getName()));
        SmartDashboard.putBoolean("Left Camera Active", LimelightHelpers.getTV(leftCamera.getName()));
        SmartDashboard.putNumber("Right Camera Tags", LimelightHelpers.getTargetCount(rightCamera.getName()));
        SmartDashboard.putNumber("Left Camera Tags", LimelightHelpers.getTargetCount(leftCamera.getName()));
        
        // Debug info for ghost robot prevention
        
    }
    
    /**
     * Get the right camera for direct access if needed
     */
    public LimelightCamera getRightCamera() {
        return rightCamera;
    }
    
    /**
     * Get the left camera for direct access if needed  
     */
    public LimelightCamera getLeftCamera() {
        return leftCamera;
    }
    
    /**
     * Manually trigger a snapshot on both cameras
     */
    public void takeSnapshot(String snapshotName) {
        LimelightHelpers.takeSnapshot(rightCamera.getName(), snapshotName + "_right");
        LimelightHelpers.takeSnapshot(leftCamera.getName(), snapshotName + "_left");
    }
    
    /**
     * Reset statistics counters and pose tracking
     */
    public void resetStatistics() {
        rightCameraMeasurements = 0;
        leftCameraMeasurements = 0;
        rejectedMeasurements = 0;
        
        // Reset timestamp tracking to prevent stale data
        lastRightCameraTimestamp = -1.0;
        lastLeftCameraTimestamp = -1.0;
        currentRightCameraPose = null;
        currentLeftCameraPose = null;
        
        // Reset odometry reset flag if needed
        hasPerformedInitialOdometryReset = false;
        
        System.out.println("Vision statistics and pose tracking reset - ghost robot prevention reinitialized");
    }
    
    /**
     * Force the initial odometry reset (for testing/debugging)
     */
    public void forceOdometryReset() {
        hasPerformedInitialOdometryReset = false;
        System.out.println("Odometry reset flag cleared - will reset on next valid vision measurement");
    }
    
    /**
     * Get the current single pose from right camera (no ghost robots)
     */
    public Pose2d getCurrentRightCameraPose() {
        return currentRightCameraPose;
    }
    
    /**
     * Get the current single pose from left camera (no ghost robots)
     */
    public Pose2d getCurrentLeftCameraPose() {
        return currentLeftCameraPose;
    }
}
