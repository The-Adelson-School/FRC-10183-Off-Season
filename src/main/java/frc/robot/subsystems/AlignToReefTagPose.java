package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToReefTagPose extends Command {
    private final SwerveSubsystem drivebase;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    
    private final boolean isRightScore;
    private final String limelightName;
    private final Timer alignmentTimer;
    private final Timer noTargetTimer;
    
    private Pose2d targetPose = null;
    private boolean hasValidTarget = false;
    
    private final ShuffleboardTab alignmentTab = Shuffleboard.getTab("Pose Alignment");
    
    // Alignment constants
    private static final double ALIGNMENT_DISTANCE = 1.2; // meters from tag
    private static final double LEFT_OFFSET = 0.6; // meters to the left of tag
    private static final double RIGHT_OFFSET = -0.6; // meters to the right of tag
    private static final double POSITION_TOLERANCE = 0.1; // meters
    private static final double ROTATION_TOLERANCE = 5.0; // degrees
    private static final double ALIGNMENT_HOLD_TIME = 0.5; // seconds
    private static final double NO_TARGET_TIMEOUT = 3.0; // seconds
    
    public AlignToReefTagPose(boolean isRightScore, SwerveSubsystem drivebase) {
        this(isRightScore, drivebase, "");
    }
    
    public AlignToReefTagPose(boolean isRightScore, SwerveSubsystem drivebase, String limelightName) {
        this.drivebase = drivebase;
        this.isRightScore = isRightScore;
        this.limelightName = limelightName.isEmpty() ? "" : limelightName; // Default limelight
        
        // Create PID controllers for pose-based alignment
        this.xController = new PIDController(2.0, 0.0, 0.1);
        this.yController = new PIDController(2.0, 0.0, 0.1);
        this.rotController = new PIDController(3.0, 0.0, 0.1);
        
        // Configure tolerances
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        
        this.alignmentTimer = new Timer();
        this.noTargetTimer = new Timer();
        
        addRequirements(drivebase);
    }
    
    @Override
    public void initialize() {
        alignmentTimer.restart();
        noTargetTimer.restart();
        targetPose = null;
        hasValidTarget = false;
        
        System.out.println("Starting pose-based alignment to " + (isRightScore ? "RIGHT" : "LEFT") + " position");
    }
    
    @Override
    public void execute() {
      
        Pose2d currentPose = drivebase.getPose();
        
  
        Pose2d detectedTargetPose = getTargetPoseFromLimelight();
        
        if (detectedTargetPose != null) {
            targetPose = detectedTargetPose;
            hasValidTarget = true;
            noTargetTimer.restart(); 
        }
       
        if (hasValidTarget && targetPose != null) {
        
            double xOutput = -xController.calculate(currentPose.getX(), targetPose.getX());
            double yOutput = -yController.calculate(currentPose.getY(), targetPose.getY());
            double rotOutput = -rotController.calculate(
                currentPose.getRotation().getRadians(), 
                targetPose.getRotation().getRadians()
            );
            
         
            xOutput = Math.max(-2.0, Math.min(2.0, xOutput));
            yOutput = Math.max(-2.0, Math.min(2.0, yOutput));
            rotOutput = Math.max(-Math.PI, Math.min(Math.PI, rotOutput));
            
    
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, rotOutput);
            drivebase.driveFieldOriented(chassisSpeeds);
            
     
            boolean isAligned = xController.atSetpoint() && 
                              yController.atSetpoint() && 
                              rotController.atSetpoint();
            
            if (isAligned) {
                if (alignmentTimer.get() == 0) {
                    alignmentTimer.restart(); // Start hold timer
                }
            } else {
                alignmentTimer.stop();
                alignmentTimer.reset(); // Reset if we lose alignment
            }
            
            // Update dashboard
            updateDashboard(currentPose, targetPose, xOutput, yOutput, rotOutput, isAligned);
            
        } else {
            // No target - stop the robot
            drivebase.drive(new ChassisSpeeds(0, 0, 0));
            updateDashboard(currentPose, null, 0, 0, 0, false);
        }
    }
    
    /**
     * Get target alignment pose from Limelight AprilTag detection
     */
    private Pose2d getTargetPoseFromLimelight() {
        // Check if we have a valid AprilTag target
        if (!LimelightHelpers.getTV(limelightName)) {
            return null;
        }
        
        // Get the fiducial ID of the detected tag
        double tagId = LimelightHelpers.getFiducialID(limelightName);
        if (tagId <= 0) {
            return null;
        }
        
        // FIXED: Use DriverStation directly instead of vision system
        boolean isRedAlliance = false;
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRedAlliance = alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        }
        
        // Get robot pose from MegaTag (alliance-aware)
        Pose2d robotPoseFromMegaTag = isRedAlliance ? 
            LimelightHelpers.getBotPose2d_wpiRed(limelightName) :
            LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
            
        if (robotPoseFromMegaTag == null) {
            return null;
        }
        
        // Get tag pose in robot coordinates
        double[] tagPoseRobotSpace = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (tagPoseRobotSpace.length < 6) {
            return null;
        }
        
        // Transform tag position to field coordinates using current robot pose
        Translation2d tagPositionRobot = new Translation2d(tagPoseRobotSpace[0], tagPoseRobotSpace[1]);
        Rotation2d tagRotationRobot = new Rotation2d(Math.toRadians(tagPoseRobotSpace[5]));
        
        Pose2d currentRobotPose = drivebase.getPose();
        Pose2d tagPoseField = currentRobotPose.transformBy(
            new Transform2d(tagPositionRobot, tagRotationRobot)
        );
        
        // Calculate alignment position relative to the tag
        double sideOffset = isRightScore ? RIGHT_OFFSET : LEFT_OFFSET;
        
        // Create alignment pose: distance away from tag, offset to side, facing tag
        Transform2d alignmentOffset = new Transform2d(
            new Translation2d(-ALIGNMENT_DISTANCE, sideOffset),
            Rotation2d.fromDegrees(0) // Face towards the tag (0 degrees relative to tag)
        );
        
        Pose2d targetAlignmentPose = tagPoseField.transformBy(alignmentOffset);
        
        // Log target calculation
        SmartDashboard.putString("Detected Tag ID", String.valueOf((int)tagId));
        SmartDashboard.putString("Tag Pose Field", 
            String.format("(%.2f, %.2f, %.1f°)", 
                tagPoseField.getX(), tagPoseField.getY(), tagPoseField.getRotation().getDegrees()));
        
        return targetAlignmentPose;
    }
    
    private void updateDashboard(Pose2d currentPose, Pose2d targetPose, 
                               double xOutput, double yOutput, double rotOutput, boolean isAligned) {
        // OPTIMIZED: Only update dashboard every few cycles to reduce loop time
        if (System.currentTimeMillis() % 200 < 20) { // Update every ~200ms instead of every cycle
            try {
                // Current pose
                alignmentTab.add("Current X", currentPose.getX());
                alignmentTab.add("Current Y", currentPose.getY());
                alignmentTab.add("Current Rotation", currentPose.getRotation().getDegrees());
                
                // Target pose
                if (targetPose != null) {
                    alignmentTab.add("Target X", targetPose.getX());
                    alignmentTab.add("Target Y", targetPose.getY());
                    alignmentTab.add("Target Rotation", targetPose.getRotation().getDegrees());
                    
                    // Errors
                    double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                    double rotationError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());
                    
                    alignmentTab.add("Distance Error", distanceError);
                    alignmentTab.add("Rotation Error", rotationError);
                }
                
                // Control outputs
                alignmentTab.add("X Output", xOutput);
                alignmentTab.add("Y Output", yOutput);
                alignmentTab.add("Rotation Output", rotOutput);
                
                // Status
                alignmentTab.add("Is Aligned", isAligned);
                alignmentTab.add("Has Valid Target", hasValidTarget);
                alignmentTab.add("Alignment Timer", alignmentTimer.get());
                alignmentTab.add("No Target Timer", noTargetTimer.get());
                alignmentTab.add("Target Side", isRightScore ? "RIGHT" : "LEFT");
                
                // Limelight status
                alignmentTab.add("Limelight Has Target", LimelightHelpers.getTV(limelightName));
                alignmentTab.add("Limelight Name", limelightName.isEmpty() ? "default" : limelightName);
            } catch (Exception e) {
                // Silently handle dashboard update errors
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds(0, 0, 0));
        System.out.println("Pose-based alignment ended. Interrupted: " + interrupted);
    }
    
    @Override
    public boolean isFinished() {
        // Finish if we've been aligned for the required time
        if (hasValidTarget && alignmentTimer.hasElapsed(ALIGNMENT_HOLD_TIME)) {
            System.out.println("Alignment successful - held position for " + ALIGNMENT_HOLD_TIME + " seconds");
            return true;
        }
        
        // Finish if we haven't seen a target for too long
        if (noTargetTimer.hasElapsed(NO_TARGET_TIMEOUT)) {
            System.out.println("Alignment failed - no target for " + NO_TARGET_TIMEOUT + " seconds");
            return true;
        }
        
        return false;
    }
}
