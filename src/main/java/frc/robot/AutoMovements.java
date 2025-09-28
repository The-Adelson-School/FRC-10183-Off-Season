package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoMovements {
    
    // Get the field layout for AprilTag positions
    private static final AprilTagFieldLayout aprilTagLayout;
    
    static {
        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
        } catch (Exception e) {
            System.err.println("Failed to load AprilTag field layout: " + e.getMessage());
            throw new RuntimeException("AprilTag field layout is required for AutoMovements", e);
        }
        aprilTagLayout = layout;
    }

    private static Pose2d getTagPose(int tagId) {
        var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
        if (tagPoseOpt.isPresent()) {
            return tagPoseOpt.get().toPose2d();
        }
        throw new RuntimeException("No AprilTag found with ID: " + tagId);
    }

    private static Transform2d createLeftOffset() {
        return new Transform2d(
            new Translation2d(-Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, Constants.FieldMovementConstants.LEFT_POSITION_OFFSET), // Left offset with distance
            new Rotation2d(Math.PI) // Face the tag
        );
    }
    
    private static Transform2d createRightOffset() {
        return new Transform2d(
            new Translation2d(-Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, -Constants.FieldMovementConstants.RIGHT_POSITION_OFFSET), // Right offset with distance
            new Rotation2d(Math.PI) // Face the tag
        );
    }
    
    private static Transform2d createAlgaeOffset() {
        return new Transform2d(
            new Translation2d(-Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 0.0), // Centered on tag with distance
            new Rotation2d(Math.PI) // Face the tag
        );
    }
    
    public enum FieldPosition {
        // RED ALLIANCE POSITIONS
        // Tag 7 positions (Red A)
        RED_A_LEFT(7, createLeftOffset()),
        RED_A_RIGHT(7, createRightOffset()),
        RED_A_Algae(7, createAlgaeOffset()),
        
        // Tag 8 positions (Red C)
        RED_C_LEFT(8, createLeftOffset()),
        RED_C_RIGHT(8, createRightOffset()),
        RED_C_Algae(8, createAlgaeOffset()),
        
        // Tag 9 positions (Red F)
        RED_F_LEFT(9, createLeftOffset()),
        RED_F_RIGHT(9, createRightOffset()),
        RED_F_Algae(9, createAlgaeOffset()),
        
        // Tag 10 positions (Red G)
        RED_G_LEFT(10, createLeftOffset()),
        RED_G_RIGHT(10, createRightOffset()),
        RED_G_Algae(10, createAlgaeOffset()),
        
        // Tag 11 positions (Red I)
        RED_I_LEFT(11, createLeftOffset()),
        RED_I_RIGHT(11, createRightOffset()),
        RED_I_Algae(11, createAlgaeOffset()),
        
        // Tag 6 positions (Red K)
        RED_K_LEFT(6, createLeftOffset()),
        RED_K_RIGHT(6, createRightOffset()),
        RED_K_Algae(6, createAlgaeOffset()),
        
        // BLUE ALLIANCE POSITIONS
        // Tag 18 positions (Blue A)
        BLUE_A_LEFT(18, createLeftOffset()),
        BLUE_A_RIGHT(18, createRightOffset()),
        BLUE_A_Algae(18, createAlgaeOffset()),
        
        // Tag 17 positions (Blue C)
        BLUE_C_LEFT(17, createLeftOffset()),
        BLUE_C_RIGHT(17, createRightOffset()),
        BLUE_C_Algae(17, createAlgaeOffset()),
        
        // Tag 22 positions (Blue E)
        BLUE_E_LEFT(22, createLeftOffset()),
        BLUE_E_RIGHT(22, createRightOffset()),
        BLUE_E_Algae(22, createAlgaeOffset()),
        
        // Tag 21 positions (Blue G)
        BLUE_G_LEFT(21, createLeftOffset()),
        BLUE_G_RIGHT(21, createRightOffset()),
        BLUE_G_Algae(21, createAlgaeOffset()),
        
        // Tag 20 positions (Blue J)
        BLUE_J_LEFT(20, createLeftOffset()),
        BLUE_J_RIGHT(20, createRightOffset()),
        BLUE_J_Algae(20, createAlgaeOffset()),
        
        // Tag 19 positions (Blue K)
        BLUE_K_LEFT(19, createLeftOffset()),
        BLUE_K_RIGHT(19, createRightOffset()),
        BLUE_K_Algae(19, createAlgaeOffset());

        private final int tagId;
        private final Transform2d offset;
        
        FieldPosition(int tagId, Transform2d offset) {
            this.tagId = tagId;
            this.offset = offset;
        }
        
        /**
         * Get the calculated pose for this position
         */
        public Pose2d getPose() {
            Pose2d tagPose2d = getTagPose(tagId);
            return tagPose2d.transformBy(offset);
        }
        
        public Translation2d getTranslation() {
            return getPose().getTranslation();
        }
        
        public Rotation2d getRotation() {
            return getPose().getRotation();
        }
        
        public int getTagId() {
            return tagId;
        }
        
        /**
         * Check if this is a left position
         */
        public boolean isLeftPosition() {
            return name().contains("_LEFT");
        }
        
        /**
         * Check if this is a right position
         */
        public boolean isRightPosition() {
            return name().contains("_RIGHT");
        }
        
        /**
         * Check if this is an Algae position (coral scoring)
         */
        public boolean isAlgaePosition() {
            return name().contains("_Algae");
        }
    }
    
    // PID Controllers for movement
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // Tolerances for position and rotation
    private final double POSITION_TOLERANCE = Constants.FieldMovementConstants.DEFAULT_POSITION_TOLERANCE;
    private final double ROTATION_TOLERANCE = Constants.FieldMovementConstants.DEFAULT_ROTATION_TOLERANCE;
    
    // Maximum speeds
    private final double MAX_TRANSLATION_SPEED = Constants.FieldMovementConstants.MAX_AUTO_TRANSLATION_SPEED;
    private final double MAX_ROTATION_SPEED = Constants.FieldMovementConstants.MAX_AUTO_ROTATION_SPEED;
    
    private final SwerveSubsystem swerveSubsystem;
    private final ShuffleboardTab autoMoveTab = Shuffleboard.getTab("Auto Movement");
    
    /**
     * Creates a new AutoMovements instance with separate X/Y PID controllers
     * 
     * @param swerveSubsystem The swerve drive subsystem
     */
    public AutoMovements(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        
        // Initialize separate PID controllers for X and Y movement
        this.xController = new PIDController(
            Constants.FieldMovementConstants.X_TRANSLATION_P, 
            Constants.FieldMovementConstants.X_TRANSLATION_I, 
            Constants.FieldMovementConstants.X_TRANSLATION_D
        );
        
        this.yController = new PIDController(
            Constants.FieldMovementConstants.Y_TRANSLATION_P, 
            Constants.FieldMovementConstants.Y_TRANSLATION_I, 
            Constants.FieldMovementConstants.Y_TRANSLATION_D
        );
        
        this.rotationController = new PIDController(
            Constants.FieldMovementConstants.ROTATION_P, 
            Constants.FieldMovementConstants.ROTATION_I, 
            Constants.FieldMovementConstants.ROTATION_D
        );
        
        // Configure PID controllers
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotationController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    /**
     * Creates a custom alignment position for any AprilTag
     */
    public static Pose2d createTagAlignmentPosition(int tagId, double distanceFromTag, double sideOffset) {
        Pose2d tagPose2d = getTagPose(tagId);
        
        // Create transform to position robot at specified distance and side offset
        Transform2d alignmentOffset = new Transform2d(
            new Translation2d(-distanceFromTag, sideOffset), // Distance back from tag with side offset
            new Rotation2d(Math.PI) // Face the tag
        );
        
        return tagPose2d.transformBy(alignmentOffset);
    }
    
    /**
     * Creates a command to align with any AprilTag at a specified distance
     */
    public Command alignWithTag(int tagId, double distanceFromTag) {
        Pose2d alignmentPose = createTagAlignmentPosition(tagId, distanceFromTag, 0.0);
        return moveToPosition(alignmentPose);
    }
    
    /**
     * Creates a command to align with an AprilTag on the left side
     */
    public Command alignWithTagLeft(int tagId) {
        return moveToPosition(createTagAlignmentPosition(tagId, 
            Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 
            Constants.FieldMovementConstants.LEFT_POSITION_OFFSET));
    }
    
    /**
     * Creates a command to align with an AprilTag on the right side
     */
    public Command alignWithTagRight(int tagId) {
        return moveToPosition(createTagAlignmentPosition(tagId, 
            Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 
            -Constants.FieldMovementConstants.RIGHT_POSITION_OFFSET));
    }
    
    /**
     * Creates a command to align with an AprilTag for algae (centered)
     */
    public Command alignWithTagAlgae(int tagId) {
        return moveToPosition(createTagAlignmentPosition(tagId, 
            Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 
            0.0));
    }
    
    /**
     * Creates a command to move the robot to a specific field position
     */
    public Command moveToPosition(FieldPosition targetPosition) {
        return moveToPosition(targetPosition.getPose());
    }
    
    /**
     * Creates a command to move the robot to a specific pose
     */
    public Command moveToPosition(Pose2d targetPose) {
        return swerveSubsystem.run(() -> {
            Pose2d currentPose = swerveSubsystem.getPose();
            
            // Calculate PID outputs for X, Y, and rotation
            double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
            double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotationOutput = rotationController.calculate(
                currentPose.getRotation().getRadians(), 
                targetPose.getRotation().getRadians()
            );
            
            // Clamp outputs to maximum speeds
            xOutput = Math.max(-MAX_TRANSLATION_SPEED, Math.min(MAX_TRANSLATION_SPEED, xOutput));
            yOutput = Math.max(-MAX_TRANSLATION_SPEED, Math.min(MAX_TRANSLATION_SPEED, yOutput));
            rotationOutput = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationOutput));
            
            // Create field-relative chassis speeds
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, rotationOutput);
            
            // Drive the robot
            swerveSubsystem.driveFieldOriented(chassisSpeeds);
            
            // Update dashboard with current status
            updateDashboard(currentPose, targetPose, xOutput, yOutput, rotationOutput);
            
        }).until(() -> isAtPosition(targetPose))
          .andThen(() -> swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0)));
    }

    /**
     * Creates a command to move to a position with custom tolerances
     */
    public Command moveToPositionWithTolerance(FieldPosition targetPosition, 
                                             double positionTolerance, 
                                             double rotationTolerance) {
        Pose2d targetPose = targetPosition.getPose();
        
        return swerveSubsystem.run(() -> {
            Pose2d currentPose = swerveSubsystem.getPose();
            
            double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
            double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotationOutput = rotationController.calculate(
                currentPose.getRotation().getRadians(), 
                targetPose.getRotation().getRadians()
            );
            
            xOutput = Math.max(-MAX_TRANSLATION_SPEED, Math.min(MAX_TRANSLATION_SPEED, xOutput));
            yOutput = Math.max(-MAX_TRANSLATION_SPEED, Math.min(MAX_TRANSLATION_SPEED, yOutput));
            rotationOutput = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationOutput));
            
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, rotationOutput);
            swerveSubsystem.driveFieldOriented(chassisSpeeds);
            
            updateDashboard(currentPose, targetPose, xOutput, yOutput, rotationOutput);
            
        }).until(() -> isAtPositionWithTolerance(targetPose, positionTolerance, rotationTolerance))
          .andThen(() -> swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0)));
    }
    
    /**
     * Creates a command that moves through multiple positions in sequence
     */
    public Command moveToMultiplePositions(FieldPosition... positions) {
        Command command = moveToPosition(positions[0]);
        
        for (int i = 1; i < positions.length; i++) {
            command = command.andThen(moveToPosition(positions[i]));
        }
        
        return command;
    }
    
    // Helper methods
    private boolean isAtPosition(Pose2d targetPose) {
        return isAtPositionWithTolerance(targetPose, POSITION_TOLERANCE, ROTATION_TOLERANCE);
    }
    
    private boolean isAtPositionWithTolerance(Pose2d targetPose, double positionTolerance, double rotationTolerance) {
        Pose2d currentPose = swerveSubsystem.getPose();
        
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
        
        if (rotationError > Math.PI) {
            rotationError = 2 * Math.PI - rotationError;
        }
        
        return distanceError <= positionTolerance && 
               rotationError <= Math.toRadians(rotationTolerance);
    }
    
    private void updateDashboard(Pose2d currentPose, Pose2d targetPose, 
                               double xOutput, double yOutput, double rotationOutput) {
        autoMoveTab.add("Current X", currentPose.getX());
        autoMoveTab.add("Current Y", currentPose.getY());
        autoMoveTab.add("Current Rotation", currentPose.getRotation().getDegrees());
        
        autoMoveTab.add("Target X", targetPose.getX());
        autoMoveTab.add("Target Y", targetPose.getY());
        autoMoveTab.add("Target Rotation", targetPose.getRotation().getDegrees());
        
        autoMoveTab.add("X Output", xOutput);
        autoMoveTab.add("Y Output", yOutput);
        autoMoveTab.add("Rotation Output", rotationOutput);
        
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.toDegrees(Math.abs(
            currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()
        ));
        
        autoMoveTab.add("Distance Error", distanceError);
        autoMoveTab.add("Rotation Error", rotationError);
        autoMoveTab.add("At Target", isAtPosition(targetPose));
    }
    
    /**
     * Updates PID constants from Shuffleboard (useful for tuning)
     */
    public void updatePIDFromDashboard() {
        var tuningTab = Shuffleboard.getTab("PID Tuning");
        
        double xTranslationP = tuningTab.add("X Translation P", Constants.FieldMovementConstants.X_TRANSLATION_P).getEntry().getDouble(Constants.FieldMovementConstants.X_TRANSLATION_P);
        double xTranslationI = tuningTab.add("X Translation I", Constants.FieldMovementConstants.X_TRANSLATION_I).getEntry().getDouble(Constants.FieldMovementConstants.X_TRANSLATION_I);
        double xTranslationD = tuningTab.add("X Translation D", Constants.FieldMovementConstants.X_TRANSLATION_D).getEntry().getDouble(Constants.FieldMovementConstants.X_TRANSLATION_D);
        
        double yTranslationP = tuningTab.add("Y Translation P", Constants.FieldMovementConstants.Y_TRANSLATION_P).getEntry().getDouble(Constants.FieldMovementConstants.Y_TRANSLATION_P);
        double yTranslationI = tuningTab.add("Y Translation I", Constants.FieldMovementConstants.Y_TRANSLATION_I).getEntry().getDouble(Constants.FieldMovementConstants.Y_TRANSLATION_I);
        double yTranslationD = tuningTab.add("Y Translation D", Constants.FieldMovementConstants.Y_TRANSLATION_D).getEntry().getDouble(Constants.FieldMovementConstants.Y_TRANSLATION_D);
        
        double rotationP = tuningTab.add("Rotation P", Constants.FieldMovementConstants.ROTATION_P).getEntry().getDouble(Constants.FieldMovementConstants.ROTATION_P);
        double rotationI = tuningTab.add("Rotation I", Constants.FieldMovementConstants.ROTATION_I).getEntry().getDouble(Constants.FieldMovementConstants.ROTATION_I);
        double rotationD = tuningTab.add("Rotation D", Constants.FieldMovementConstants.ROTATION_D).getEntry().getDouble(Constants.FieldMovementConstants.ROTATION_D);
        
        xController.setPID(xTranslationP, xTranslationI, xTranslationD);
        yController.setPID(yTranslationP, yTranslationI, yTranslationD);
        rotationController.setPID(rotationP, rotationI, rotationD);
    }
    
    // Utility methods
    public double getDistanceToPosition(FieldPosition targetPosition) {
        Pose2d currentPose = swerveSubsystem.getPose();
        return currentPose.getTranslation().getDistance(targetPosition.getTranslation());
    }
    
    public FieldPosition getClosestPosition() {
        Pose2d currentPose = swerveSubsystem.getPose();
        FieldPosition closest = FieldPosition.RED_A_LEFT;
        double closestDistance = Double.MAX_VALUE;
        
        for (FieldPosition position : FieldPosition.values()) {
            double distance = currentPose.getTranslation().getDistance(position.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = position;
            }
        }
        
        return closest;
    }
    
    public Pose2d getCurrentPosition() {
        return swerveSubsystem.getPose();
    }
    
    public static Pose2d createCustomPosition(double x, double y, double headingDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
    }
    
    /**
     * Gets information about a specific AprilTag alignment position
     */
    public String getPositionInfo(FieldPosition position) {
        String positionType = position.isLeftPosition() ? "Left" : 
                             position.isRightPosition() ? "Right" :
                             position.isAlgaePosition() ? "Algae" : "Unknown";
        return String.format("Position: %s, Tag ID: %d, Type: %s, Distance: %.1fm, Pose: (%.2f, %.2f, %.1f°)",
            position.name(),
            position.getTagId(),
            positionType,
            Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG,
            position.getPose().getX(),
            position.getPose().getY(),
            position.getPose().getRotation().getDegrees());
    }
}
