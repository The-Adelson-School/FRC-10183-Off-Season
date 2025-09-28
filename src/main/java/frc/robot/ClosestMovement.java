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
     * Gets the closest algae position based on current robot location and alliance
     */
    public FieldPosition getClosestAlgaePosition() {
        Pose2d currentPose = swerveSubsystem.getPose();
        boolean isRedAlliance = isRedAlliance();
        
        FieldPosition closestAlgae = null;
        double closestDistance = Double.MAX_VALUE;
        
        // Filter positions based on alliance and algae type
        for (FieldPosition position : FieldPosition.values()) {
            // Check if position matches our alliance and is an algae position
            if (isPositionForAlliance(position, isRedAlliance) && position.isAlgaePosition()) {
                double distance = currentPose.getTranslation().getDistance(position.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestAlgae = position;
                }
            }
        }
        
        // Fallback if no position found
        if (closestAlgae == null) {
            closestAlgae = isRedAlliance ? FieldPosition.RED_A_Algae : FieldPosition.BLUE_A_Algae;
        }
        
        return closestAlgae;
    }
    
    /**
     * Gets the closest left position based on current robot location and alliance
     */
    public FieldPosition getClosestLeftPosition() {
        Pose2d currentPose = swerveSubsystem.getPose();
        boolean isRedAlliance = isRedAlliance();
        
        FieldPosition closestLeft = null;
        double closestDistance = Double.MAX_VALUE;
        
        // Filter positions based on alliance and left type
        for (FieldPosition position : FieldPosition.values()) {
            // Check if position matches our alliance and is a left position
            if (isPositionForAlliance(position, isRedAlliance) && position.isLeftPosition()) {
                double distance = currentPose.getTranslation().getDistance(position.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestLeft = position;
                }
            }
        }
        
        // Fallback if no position found
        if (closestLeft == null) {
            closestLeft = isRedAlliance ? FieldPosition.RED_A_LEFT : FieldPosition.BLUE_A_LEFT;
        }
        
        return closestLeft;
    }
    
    /**
     * Gets the closest right position based on current robot location and alliance
     */
    public FieldPosition getClosestRightPosition() {
        Pose2d currentPose = swerveSubsystem.getPose();
        boolean isRedAlliance = isRedAlliance();
        
        FieldPosition closestRight = null;
        double closestDistance = Double.MAX_VALUE;
        
        // Filter positions based on alliance and right type
        for (FieldPosition position : FieldPosition.values()) {
            // Check if position matches our alliance and is a right position
            if (isPositionForAlliance(position, isRedAlliance) && position.isRightPosition()) {
                double distance = currentPose.getTranslation().getDistance(position.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestRight = position;
                }
            }
        }
        
        // Fallback if no position found
        if (closestRight == null) {
            closestRight = isRedAlliance ? FieldPosition.RED_A_RIGHT : FieldPosition.BLUE_A_RIGHT;
        }
        
        return closestRight;
    }

    /**
     * Command to move to the closest right position
     */
    public Command moveToClosestRightPosition() {
        return autoMovements.moveToPosition(getClosestRightPosition())
            .withName("MoveToClosestRight");
    }
    
    /**
     * Command to move to the closest left position  
     */
    public Command moveToClosestLeftPosition() {
        return autoMovements.moveToPosition(getClosestLeftPosition())
            .withName("MoveToClosestLeft");
    }
    
    /**
     * Command to move to the closest algae position
     */
    public Command moveToClosestAlgaePosition() {
        return autoMovements.moveToPosition(getClosestAlgaePosition())
            .withName("MoveToClosestAlgae");
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
        FieldPosition closestAlgae = getClosestAlgaePosition();
        boolean isRed = isRedAlliance();
        
        return String.format("Alliance: %s | Closest Left: %s | Closest Right: %s | Closest Algae: %s",
            isRed ? "RED" : "BLUE",
            closestLeft.name(),
            closestRight.name(),
            closestAlgae.name());
    }
}
