package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private boolean isSecondLimelight;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = 1;
  private String limelightName;
  private final ShuffleboardTab alignmentTab = Shuffleboard.getTab("Alignment");

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    this(isRightScore, drivebase, false);
  }

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase, boolean isSecondLimelight) {
    xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);  // Vertical movement
    yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);  // Horitontal movement
    rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0.0);  // Rotation
    this.isRightScore = isRightScore;
    this.isSecondLimelight = isSecondLimelight;
    this.drivebase = drivebase;
    this.limelightName = isSecondLimelight ? "limelight-two" : "";
    addRequirements(drivebase);
  }

  private boolean isTagIDAllowed(double detectedTagID) {
    // Since we removed tag filtering, all tags are now allowed
    return true;
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    if (isSecondLimelight) {
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_2);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT_2);

      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT_2);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT_2);

      yController.setSetpoint(isRightScore ? VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_2 : -VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_2);
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT_2);
    } else {
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

      yController.setSetpoint(isRightScore ? VisionConstants.Y_SETPOINT_REEF_ALIGNMENT : -VisionConstants.Y_SETPOINT_REEF_ALIGNMENT);
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);
    }

    tagID = LimelightHelpers.getFiducialID(limelightName);
  }

  @Override
  public void execute() {
    double detectedTagID = LimelightHelpers.getFiducialID(limelightName);
    
    if (LimelightHelpers.getTV(limelightName) && detectedTagID == tagID && isTagIDAllowed(detectedTagID)) {
      this.dontSeeTagTimer.reset();
      
      double[] postions = LimelightHelpers.getBotPose_TargetSpace(limelightName);
      alignmentTab.add("X Position", postions[2]);

      double xSpeed = -xController.calculate(postions[2]);
      alignmentTab.add("X Speed", xSpeed);
      
      // For second limelight (opposite side), invert movement directions
      double ySpeed, rotValue;
      if (isSecondLimelight) {
        ySpeed = -yController.calculate(postions[0]);  // Inverted for opposite direction
        rotValue = -rotController.calculate(postions[4]);  // Inverted for opposite direction
      } else {
        ySpeed = yController.calculate(postions[0]);
        rotValue = rotController.calculate(postions[4]);
      }

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    alignmentTab.add("Pose Valid Timer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
      // Check if the robot is within the acceptable tolerances for alignment
      boolean isAligned = rotController.atSetpoint() &&
                          yController.atSetpoint() &&
                          xController.atSetpoint();
  
      // Log alignment status for debugging
      alignmentTab.add("Is Aligned", isAligned);
      alignmentTab.add("Stop Timer", stopTimer.get());
      alignmentTab.add("Dont See Tag Timer Elapsed", dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME));
  
      // Stop if the robot is aligned for the required time or if the timers have elapsed
      return (isAligned && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME)) ||
             dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME);
  }

}