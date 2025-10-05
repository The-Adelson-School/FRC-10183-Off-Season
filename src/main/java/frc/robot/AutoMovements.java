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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoMovements {
    
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
            new Translation2d(-Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, Constants.FieldMovementConstants.LEFT_POSITION_OFFSET),
            new Rotation2d(Math.PI)
        );
    }
    
    private static Transform2d createRightOffset() {
        return new Transform2d(
            new Translation2d(-Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, -Constants.FieldMovementConstants.RIGHT_POSITION_OFFSET),
            new Rotation2d(Math.PI)
        );
    }
    
    private static Transform2d createAlgaeOffset() {
        return new Transform2d(
            new Translation2d(-Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 0.0),
            new Rotation2d(Math.PI)
        );
    }
    
    public enum FieldPosition {
        RED_A_LEFT(7, createLeftOffset()),
        RED_A_RIGHT(7, createRightOffset()),
        RED_A_Algae(7, createAlgaeOffset()),
        
        RED_C_LEFT(8, createLeftOffset()),
        RED_C_RIGHT(8, createRightOffset()),
        RED_C_Algae(8, createAlgaeOffset()),
        
        RED_F_LEFT(9, createLeftOffset()),
        RED_F_RIGHT(9, createRightOffset()),
        RED_F_Algae(9, createAlgaeOffset()),
        
        RED_G_LEFT(10, createLeftOffset()),
        RED_G_RIGHT(10, createRightOffset()),
        RED_G_Algae(10, createAlgaeOffset()),
        
        RED_I_LEFT(11, createLeftOffset()),
        RED_I_RIGHT(11, createRightOffset()),
        RED_I_Algae(11, createAlgaeOffset()),
        
        RED_K_LEFT(6, createLeftOffset()),
        RED_K_RIGHT(6, createRightOffset()),
        RED_K_Algae(6, createAlgaeOffset()),
        
        BLUE_A_LEFT(18, createLeftOffset()),
        BLUE_A_RIGHT(18, createRightOffset()),
        BLUE_A_Algae(18, createAlgaeOffset()),
        
        BLUE_C_LEFT(17, createLeftOffset()),
        BLUE_C_RIGHT(17, createRightOffset()),
        BLUE_C_Algae(17, createAlgaeOffset()),
        
        BLUE_E_LEFT(22, createLeftOffset()),
        BLUE_E_RIGHT(22, createRightOffset()),
        BLUE_E_Algae(22, createAlgaeOffset()),
        
        BLUE_G_LEFT(21, createLeftOffset()),
        BLUE_G_RIGHT(21, createRightOffset()),
        BLUE_G_Algae(21, createAlgaeOffset()),
        
        BLUE_J_LEFT(20, createLeftOffset()),
        BLUE_J_RIGHT(20, createRightOffset()),
        BLUE_J_Algae(20, createAlgaeOffset()),
        
        BLUE_K_LEFT(19, createLeftOffset()),
        BLUE_K_RIGHT(19, createRightOffset()),
        BLUE_K_Algae(19, createAlgaeOffset());

        private final int tagId;
        private final Transform2d offset;
        
        FieldPosition(int tagId, Transform2d offset) {
            this.tagId = tagId;
            this.offset = offset;
        }
        
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
        
        public boolean isLeftPosition() {
            return name().contains("_LEFT");
        }
        
        public boolean isRightPosition() {
            return name().contains("_RIGHT");
        }
        
        public boolean isAlgaePosition() {
            return name().contains("_Algae");
        }
    }
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    private final double POSITION_TOLERANCE = Constants.FieldMovementConstants.DEFAULT_POSITION_TOLERANCE;
    private final double ROTATION_TOLERANCE = Constants.FieldMovementConstants.DEFAULT_ROTATION_TOLERANCE;
    
    private final double MAX_TRANSLATION_SPEED = Constants.FieldMovementConstants.MAX_AUTO_TRANSLATION_SPEED;
    private final double MAX_ROTATION_SPEED = Constants.FieldMovementConstants.MAX_AUTO_ROTATION_SPEED;
    
    private final SwerveSubsystem swerveSubsystem;
    private final ShuffleboardTab autoMoveTab = Shuffleboard.getTab("Auto Movement");
    
    // Pre-create dashboard entries
    private final GenericEntry currentXEntry = autoMoveTab.add("Current X", 0.0).getEntry();
    private final GenericEntry currentYEntry = autoMoveTab.add("Current Y", 0.0).getEntry();
    private final GenericEntry currentRotationEntry = autoMoveTab.add("Current Rotation", 0.0).getEntry();
    private final GenericEntry targetXEntry = autoMoveTab.add("Target X", 0.0).getEntry();
    private final GenericEntry targetYEntry = autoMoveTab.add("Target Y", 0.0).getEntry();
    private final GenericEntry targetRotationEntry = autoMoveTab.add("Target Rotation", 0.0).getEntry();
    private final GenericEntry xOutputEntry = autoMoveTab.add("X Output", 0.0).getEntry();
    private final GenericEntry yOutputEntry = autoMoveTab.add("Y Output", 0.0).getEntry();
    private final GenericEntry rotationOutputEntry = autoMoveTab.add("Rotation Output", 0.0).getEntry();
    private final GenericEntry distanceErrorEntry = autoMoveTab.add("Distance Error", 0.0).getEntry();
    private final GenericEntry rotationErrorEntry = autoMoveTab.add("Rotation Error", 0.0).getEntry();
    private final GenericEntry atTargetEntry = autoMoveTab.add("At Target", false).getEntry();
    
    public AutoMovements(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        
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
        
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotationController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    public static Pose2d createTagAlignmentPosition(int tagId, double distanceFromTag, double sideOffset) {
        Pose2d tagPose2d = getTagPose(tagId);
        
        Transform2d alignmentOffset = new Transform2d(
            new Translation2d(-distanceFromTag, sideOffset),
            new Rotation2d(Math.PI)
        );
        
        return tagPose2d.transformBy(alignmentOffset);
    }
    
    public Command alignWithTag(int tagId, double distanceFromTag) {
        Pose2d alignmentPose = createTagAlignmentPosition(tagId, distanceFromTag, 0.0);
        return moveToPosition(alignmentPose);
    }
    
    public Command alignWithTagLeft(int tagId) {
        return moveToPosition(createTagAlignmentPosition(tagId, 
            Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 
            Constants.FieldMovementConstants.LEFT_POSITION_OFFSET));
    }
    
    public Command alignWithTagRight(int tagId) {
        return moveToPosition(createTagAlignmentPosition(tagId, 
            Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 
            -Constants.FieldMovementConstants.RIGHT_POSITION_OFFSET));
    }
    
    public Command alignWithTagAlgae(int tagId) {
        return moveToPosition(createTagAlignmentPosition(tagId, 
            Constants.FieldMovementConstants.POSITION_DISTANCE_FROM_TAG, 
            0.0));
    }
    
    public Command moveToPosition(FieldPosition targetPosition) {
        return moveToPosition(targetPosition.getPose());
    }
    
    public Command moveToPosition(Pose2d targetPose) {
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
            
        }).until(() -> isAtPosition(targetPose))
          .andThen(() -> swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0)));
    }

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
    
    public Command moveToMultiplePositions(FieldPosition... positions) {
        Command command = moveToPosition(positions[0]);
        
        for (int i = 1; i < positions.length; i++) {
            command = command.andThen(moveToPosition(positions[i]));
        }
        
        return command;
    }
    
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
        currentXEntry.setDouble(currentPose.getX());
        currentYEntry.setDouble(currentPose.getY());
        currentRotationEntry.setDouble(currentPose.getRotation().getDegrees());
        
        targetXEntry.setDouble(targetPose.getX());
        targetYEntry.setDouble(targetPose.getY());
        targetRotationEntry.setDouble(targetPose.getRotation().getDegrees());
        
        xOutputEntry.setDouble(xOutput);
        yOutputEntry.setDouble(yOutput);
        rotationOutputEntry.setDouble(rotationOutput);
        
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.toDegrees(Math.abs(
            currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()
        ));
        
        distanceErrorEntry.setDouble(distanceError);
        rotationErrorEntry.setDouble(rotationError);
        atTargetEntry.setBoolean(isAtPosition(targetPose));
    }
    
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
