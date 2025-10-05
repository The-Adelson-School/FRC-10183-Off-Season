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

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);  // Vertical movement
    yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);  // Horitontal movement
    rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0.0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? VisionConstants.Y_SETPOINT_REEF_ALIGNMENT : -VisionConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-right");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-right") && LimelightHelpers.getFiducialID("limelight-right") == tagID) {
      this.dontSeeTagTimer.reset();
      

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
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
      SmartDashboard.putBoolean("Is Aligned", isAligned);
      SmartDashboard.putNumber("Stop Timer", stopTimer.get());
      SmartDashboard.putBoolean("Dont See Tag Timer Elapsed", dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME));
  
      // Stop if the robot is aligned for the required time or if the timers have elapsed
      return (isAligned && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME)) ||
             dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME);
  }

}