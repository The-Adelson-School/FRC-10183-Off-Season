package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.Optional;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
    
    // Statistics tracking
    private int rightCameraMeasurements = 0;
    private int leftCameraMeasurements = 0;
    private int rejectedMeasurements = 0;
    
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
        this.poseConsumer = poseConsumer;
        
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
     */
    public void updateRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
        if (VisionConstants.ENABLE_MEGATAG2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                rightCamera.getName(), 
                yawDegrees, 
                yawRateDegPerSec, 
                VisionConstants.PITCH_DEGREES, 
                VisionConstants.PITCH_RATE_DEG_PER_SEC, 
                VisionConstants.ROLL_DEGREES, 
                VisionConstants.ROLL_RATE_DEG_PER_SEC
            );
            LimelightHelpers.SetRobotOrientation_NoFlush(
                leftCamera.getName(), 
                yawDegrees, 
                yawRateDegPerSec, 
                VisionConstants.PITCH_DEGREES, 
                VisionConstants.PITCH_RATE_DEG_PER_SEC, 
                VisionConstants.ROLL_DEGREES, 
                VisionConstants.ROLL_RATE_DEG_PER_SEC
            );
            LimelightHelpers.Flush(); // Flush all updates at once
        }
    }
    
    /**
     * Process vision measurements from all cameras and update odometry
     * Call this method periodically (in robot periodic or subsystem periodic)
     */
    public void processVisionMeasurements() {
        // Process each camera
        processCameraMeasurements(rightCamera);
        processCameraMeasurements(leftCamera);
        
        // Update dashboard statistics
        updateDashboard();
    }
    
    /**
     * Process measurements from a specific camera
     */
    private void processCameraMeasurements(LimelightCamera camera) {
        try {
            // Get pose estimate using blue alliance coordinates (recommended)
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera.getName());
            
            if (!LimelightHelpers.validPoseEstimate(poseEstimate)) {
                return; // No valid measurement
            }
            
            // Validate the measurement
            if (isValidMeasurement(poseEstimate, camera)) {
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
                poseConsumer.accept(measurement);
                
                // Update statistics
                if (camera == rightCamera) {
                    rightCameraMeasurements++;
                } else {
                    leftCameraMeasurements++;
                }
                
                // Debug output - shows active vision integration
                debugTab.add("Last Measurement Source", camera.getName());
                debugTab.add("Last Measurement Confidence", confidence);
                debugTab.add("Last Measurement Tags", poseEstimate.tagCount);
                debugTab.add("Last Measurement Distance", poseEstimate.avgTagDist);
                
                // Log successful measurement processing
                System.out.println(String.format("Vision Update: %s - Tags: %d, Confidence: %.3f, Distance: %.2fm", 
                    camera.getName(), poseEstimate.tagCount, confidence, poseEstimate.avgTagDist));
                    
            } else {
                rejectedMeasurements++;
                // Debug rejected measurements
                debugTab.add("Last Rejected Reason", getRejectionReason(poseEstimate, camera));
            }
            
        } catch (Exception e) {
            DriverStation.reportError("Vision processing error for " + camera.getName() + ": " + e.getMessage(), false);
        }
    }
    
    /**
     * Validate if a pose estimate should be accepted
     */
    private boolean isValidMeasurement(PoseEstimate poseEstimate, LimelightCamera camera) {
        // Check if we have valid fiducials
        if (poseEstimate.rawFiducials == null || poseEstimate.rawFiducials.length == 0) {
            return false;
        }
        
        // Check Z-axis error (pose height should be reasonable for a ground robot)
        if (Math.abs(poseEstimate.pose.getY()) > VisionConstants.MAX_Z_ERROR) {
            return false; // Reject if robot appears to be floating or underground
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
        visionTab.add("Right Camera Measurements", rightCameraMeasurements);
        visionTab.add("Left Camera Measurements", leftCameraMeasurements);
        visionTab.add("Rejected Measurements", rejectedMeasurements);
        visionTab.add("Total Measurements", rightCameraMeasurements + leftCameraMeasurements);
        visionTab.add("Acceptance Rate (%)", 
            (rightCameraMeasurements + leftCameraMeasurements) / 
            Math.max(1.0, rightCameraMeasurements + leftCameraMeasurements + rejectedMeasurements) * 100.0);
        
        // Current target information for each camera
        visionTab.add("Right Camera Has Target", LimelightHelpers.getTV(rightCamera.getName()));
        visionTab.add("Left Camera Has Target", LimelightHelpers.getTV(leftCamera.getName()));
        visionTab.add("Right Camera Tag Count", LimelightHelpers.getTargetCount(rightCamera.getName()));
        visionTab.add("Left Camera Tag Count", LimelightHelpers.getTargetCount(leftCamera.getName()));
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
     * Reset statistics counters
     */
    public void resetStatistics() {
        rightCameraMeasurements = 0;
        leftCameraMeasurements = 0;
        rejectedMeasurements = 0;
    }
}
