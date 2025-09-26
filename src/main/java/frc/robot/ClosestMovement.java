package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoMovements.FieldPosition;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClosestMovement {
    
    private final AutoMovements autoMovements;
    private final SwerveSubsystem swerveSubsystem;
    
    public ClosestMovement(AutoMovements autoMovements, SwerveSubsystem swerveSubsystem) {
        this.autoMovements = autoMovements;
        this.swerveSubsystem = swerveSubsystem;
    }
    
    /**
     * Gets the closest LEFT field position based on current robot location and alliance
     */
    public FieldPosition getClosestLeftPosition() {
        Pose2d currentPose = swerveSubsystem.getPose();
        boolean isRedAlliance = isRedAlliance();
        
        FieldPosition closestLeft = null;
        double closestDistance = Double.MAX_VALUE;
        
        // Filter positions based on alliance and left side
        for (FieldPosition position : FieldPosition.values()) {
            // Check if position matches our alliance and is a left position
            if (isPositionForAlliance(position, isRedAlliance) && position.isLeftSide()) {
                double distance = currentPose.getTranslation().getDistance(position.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestLeft = position;
                }
            }
        }
        
        // Fallback if no position found
        if (closestLeft == null) {
            closestLeft = isRedAlliance ? FieldPosition.RED_A : FieldPosition.BLUE_A;
        }
        
        return closestLeft;
    }
    
    /**
     * Gets the closest RIGHT field position based on current robot location and alliance
     */
    public FieldPosition getClosestRightPosition() {
        Pose2d currentPose = swerveSubsystem.getPose();
        boolean isRedAlliance = isRedAlliance();
        
        FieldPosition closestRight = null;
        double closestDistance = Double.MAX_VALUE;
        
        // Filter positions based on alliance and right side
        for (FieldPosition position : FieldPosition.values()) {
            // Check if position matches our alliance and is a right position
            if (isPositionForAlliance(position, isRedAlliance) && !position.isLeftSide()) {
                double distance = currentPose.getTranslation().getDistance(position.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestRight = position;
                }
            }
        }
        
        // Fallback if no position found
        if (closestRight == null) {
            closestRight = isRedAlliance ? FieldPosition.RED_B : FieldPosition.BLUE_B;
        }
        
        return closestRight;
    }
    
    /**
     * Command to move to the closest LEFT position
     */
    public Command moveToClosestLeftPosition() {
        return autoMovements.moveToPosition(getClosestLeftPosition())
            .withName("MoveToClosestLeft");
    }
    
    /**
     * Command to move to the closest RIGHT position
     */
    public Command moveToClosestRightPosition() {
        return autoMovements.moveToPosition(getClosestRightPosition())
            .withName("MoveToClosestRight");
    }
    
    /**
     * Command to move to the closest LEFT L1 position
     */
    public Command moveToClosestLeftL1Position() {
        FieldPosition leftPos = getClosestLeftPosition();
        FieldPosition leftL1Pos = getL1VariantOfPosition(leftPos, true);
        return autoMovements.moveToPosition(leftL1Pos)
            .withName("MoveToClosestLeftL1");
    }
    
    /**
     * Command to move to the closest RIGHT L1 position
     */
    public Command moveToClosestRightL1Position() {
        FieldPosition rightPos = getClosestRightPosition();
        FieldPosition rightL1Pos = getL1VariantOfPosition(rightPos, false);
        return autoMovements.moveToPosition(rightL1Pos)
            .withName("MoveToClosestRightL1");
    }
    
    /**
     * Helper method to check if a position belongs to the current alliance
     */
    private boolean isPositionForAlliance(FieldPosition position, boolean isRedAlliance) {
        String positionName = position.name();
        if (isRedAlliance) {
            return positionName.startsWith("RED_");
        } else {
            return positionName.startsWith("BLUE_");
        }
    }
    
    /**
     * Helper method to get the L1 variant of a coral position
     */
    private FieldPosition getL1VariantOfPosition(FieldPosition position, boolean isLeft) {
        String positionName = position.name();
        
        // Remove any existing suffix and add L1 suffix
        if (positionName.contains("_LEFTL1") || positionName.contains("_RIGHTL1")) {
            // Already an L1 position
            return position;
        }
        
        // Convert coral position to L1 position
        String baseName = positionName; // e.g., "RED_A", "BLUE_C"
        String l1Suffix = isLeft ? "_LEFTL1" : "_RIGHTL1";
        String l1PositionName = baseName + l1Suffix;
        
        // Find the corresponding L1 position
        for (FieldPosition pos : FieldPosition.values()) {
            if (pos.name().equals(l1PositionName)) {
                return pos;
            }
        }
        
        // Fallback to original position if L1 variant not found
        return position;
    }
    
    /**
     * Checks if the alliance is red
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
    
    /**
     * Get information about the closest positions
     */
    public String getClosestPositionsInfo() {
        FieldPosition closestLeft = getClosestLeftPosition();
        FieldPosition closestRight = getClosestRightPosition();
        boolean isRed = isRedAlliance();
        
        return String.format("Alliance: %s | Closest Left: %s | Closest Right: %s",
            isRed ? "RED" : "BLUE",
            closestLeft.name(),
            closestRight.name());
    }
}
