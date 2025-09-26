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
    

    private static Transform2d createCoralOffset(boolean isLeft) {
        double sideOffset = isLeft ? Constants.FieldMovementConstants.LEFT_SIDE_OFFSET : -Constants.FieldMovementConstants.RIGHT_SIDE_OFFSET;
        return new Transform2d(
            new Translation2d(-Constants.FieldMovementConstants.CORAL_OFFSET_DISTANCE, sideOffset),
            new Rotation2d(Math.PI) // Face the tag
        );
    }
    
    private static Transform2d createL1Offset(boolean isLeft) {
        double sideOffset = isLeft ? Constants.FieldMovementConstants.LEFT_SIDE_OFFSET : -Constants.FieldMovementConstants.RIGHT_SIDE_OFFSET;
        return new Transform2d(
            new Translation2d(-Constants.FieldMovementConstants.L1_OFFSET_DISTANCE, sideOffset),
            new Rotation2d(Math.PI) // Face the tag
        );
    }
    
    public enum FieldPosition {
        // RED ALLIANCE POSITIONS
        // Tag 7 positions (Red A,B)
        RED_A(7, true, createCoralOffset(true)),
        RED_A_LEFTL1(7, true, createL1Offset(true)),
        RED_A_RIGHTL1(7, false, createL1Offset(false)),
        RED_B(7, false, createCoralOffset(false)),
        
        // Tag 8 positions (Red C,D)
        RED_C(8, true, createCoralOffset(true)),
        RED_C_LEFTL1(8, true, createL1Offset(true)),
        RED_C_RIGHTL1(8, false, createL1Offset(false)),
        RED_D(8, false, createCoralOffset(false)),
        
        // Tag 9 positions (Red F,E)
        RED_F(9, true, createCoralOffset(true)),
        RED_F_LEFTL1(9, true, createL1Offset(true)),
        RED_F_RIGHTL1(9, false, createL1Offset(false)),
        RED_E(9, false, createCoralOffset(false)),
        
        // Tag 10 positions (Red G,H)
        RED_G(10, true, createCoralOffset(true)),
        RED_G_LEFTL1(10, true, createL1Offset(true)),
        RED_G_RIGHTL1(10, false, createL1Offset(false)),
        RED_H(10, false, createCoralOffset(false)),
        
        // Tag 11 positions (Red I,J)
        RED_I(11, true, createCoralOffset(true)),
        RED_I_LEFTL1(11, true, createL1Offset(true)),
        RED_I_RIGHTL1(11, false, createL1Offset(false)),
        RED_J(11, false, createCoralOffset(false)),
        
        // Tag 6 positions (Red K,L)
        RED_K(6, true, createCoralOffset(true)),
        RED_K_LEFTL1(6, true, createL1Offset(true)),
        RED_K_RIGHTL1(6, false, createL1Offset(false)),
        RED_L(6, false, createCoralOffset(false)),
        
        // BLUE ALLIANCE POSITIONS
        // Tag 18 positions (Blue A,B)
        BLUE_A(18, true, createCoralOffset(true)),
        BLUE_A_LEFTL1(18, true, createL1Offset(true)),
        BLUE_A_RIGHTL1(18, false, createL1Offset(false)),
        BLUE_B(18, false, createCoralOffset(false)),
        
        // Tag 17 positions (Blue C,D)
        BLUE_C(17, true, createCoralOffset(true)),
        BLUE_C_LEFTL1(17, true, createL1Offset(true)),
        BLUE_C_RIGHTL1(17, false, createL1Offset(false)),
        BLUE_D(17, false, createCoralOffset(false)),
        
        // Tag 22 positions (Blue E,F)
        BLUE_E(22, true, createCoralOffset(true)),
        BLUE_E_LEFTL1(22, true, createL1Offset(true)),
        BLUE_E_RIGHTL1(22, false, createL1Offset(false)),
        BLUE_F(22, false, createCoralOffset(false)),
        
        // Tag 21 positions (Blue G,H)
        BLUE_G(21, true, createCoralOffset(true)),
        BLUE_G_LEFTL1(21, true, createL1Offset(true)),
        BLUE_G_RIGHTL1(21, false, createL1Offset(false)),
        BLUE_H(21, false, createCoralOffset(false)),
        
        // Tag 20 positions (Blue J,I)
        BLUE_J(20, true, createCoralOffset(true)),
        BLUE_J_LEFTL1(20, true, createL1Offset(true)),
        BLUE_J_RIGHTL1(20, false, createL1Offset(false)),
        BLUE_I(20, false, createCoralOffset(false)),
        
        // Tag 19 positions (Blue K,L)
        BLUE_K(19, true, createCoralOffset(true)),
        BLUE_K_LEFTL1(19, true, createL1Offset(true)),
        BLUE_K_RIGHTL1(19, false, createL1Offset(false)),
        BLUE_L(19, false, createCoralOffset(false));

        private final int tagId;
        private final boolean isLeft;
        private final Transform2d offset;
        
        FieldPosition(int tagId, boolean isLeft, Transform2d offset) {
            this.tagId = tagId;
            this.isLeft = isLeft;
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
        
        public boolean isLeftSide() {
            return isLeft;
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
            new Translation2d(-distanceFromTag, sideOffset),
            new Rotation2d(Math.PI) // Face the tag
        );
        
        return tagPose2d.transformBy(alignmentOffset);
    }
    
    /**
     * Creates a command to align with any AprilTag at a specified distance and offset
     */
    public Command alignWithTag(int tagId, double distanceFromTag, double sideOffset) {
        Pose2d alignmentPose = createTagAlignmentPosition(tagId, distanceFromTag, sideOffset);
        return moveToPosition(alignmentPose);
    }
    
    /**
     * Creates a command to align with an AprilTag using coral offset
     */
    public Command alignWithTagCoral(int tagId, boolean leftSide) {
        double sideOffset = leftSide ? Constants.FieldMovementConstants.LEFT_SIDE_OFFSET : -Constants.FieldMovementConstants.RIGHT_SIDE_OFFSET;
        return alignWithTag(tagId, Constants.FieldMovementConstants.CORAL_OFFSET_DISTANCE, sideOffset);
    }
    
    /**
     * Creates a command to align with an AprilTag using L1 offset
     */
    public Command alignWithTagL1(int tagId, boolean leftSide) {
        double sideOffset = leftSide ? Constants.FieldMovementConstants.LEFT_SIDE_OFFSET : -Constants.FieldMovementConstants.RIGHT_SIDE_OFFSET;
        return alignWithTag(tagId, Constants.FieldMovementConstants.L1_OFFSET_DISTANCE, sideOffset);
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
        FieldPosition closest = FieldPosition.RED_A;
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
        return String.format("Position: %s, Tag ID: %d, Side: %s, Pose: (%.2f, %.2f, %.1f°)",
            position.name(),
            position.getTagId(),
            position.isLeftSide() ? "Left" : "Right",
            position.getPose().getX(),
            position.getPose().getY(),
            position.getPose().getRotation().getDegrees());
    }
}
