package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;
import java.util.Arrays;

public class AlignToReefNew extends Command {
  private final PIDController xController, yController, rotController;
  private final boolean isRightScore;
  private final SwerveSubsystem drivebase;

  private static final String LIMELIGHTLEFT = "limelight-left";
  private static final String LIMELIGHTRIGHT = "limelight-right";

  // Timers & state
  private final Timer dontSeeTagTimer = new Timer();
  private final Timer stopTimer = new Timer();
  private boolean alignmentCompleted = false;

  // Driver override
  private final DoubleSupplier leftYSupplier;
  private final DoubleSupplier leftXSupplier;
  private final DoubleSupplier rightXSupplier;
  private static final double DRIVER_INPUT_THRESHOLD = 0.1;
  private boolean driverOverride = false;

  // Shooter (no algae)
  private final ElevatorSubsystem elevator;
  private final Timer shooterTimer = new Timer();
  private boolean shooterActivated = false;
  private static final double SHOOTER_RUN_TIME = 0.5; // seconds

  // ---------- Constructors ----------
  public AlignToReefNew(
      boolean isRightScore,
      SwerveSubsystem drivebase,
      DoubleSupplier leftY,
      DoubleSupplier leftX,
      DoubleSupplier rightX,
      ElevatorSubsystem elevator) {

    this.isRightScore = isRightScore;
    this.drivebase = drivebase;

    this.leftYSupplier = leftY;
    this.leftXSupplier = leftX;
    this.rightXSupplier = rightX;
    this.elevator = elevator;

    xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);
    yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
    rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0);

    // Choose setpoints based only on desired scoring side (no camera-specific offsets)
    if (isRightScore) {
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_RIGHT);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT_RIGHT);

      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT_RIGHT);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT_RIGHT);

      yController.setSetpoint(Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT));
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT_RIGHT);
    } else {
      rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT_LEFT);
      rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT_LEFT);

      xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT_LEFT);
      xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT_LEFT);

      yController.setSetpoint(Math.abs(VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT));
      yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT_LEFT);
    }

    addRequirements(drivebase);
  }

  // ---------- Lifecycle ----------
  @Override
  public void initialize() {
    stopTimer.restart();
    dontSeeTagTimer.restart();

    driverOverride = false;
    shooterActivated = false;
    alignmentCompleted = false;

    System.out.println(
        "Reef alignment started | Side: " + (isRightScore ? "RIGHT" : "LEFT"));
  }

  @Override
  public void execute() {
    // Driver override -> stop & exit execute early
    if (checkDriverOverride()) {
      driverOverride = true;
      drivebase.drive(new Translation2d(), 0, false);
      System.out.println("ALIGNMENT STOPPED: Driver override detected");
      return;
    }

    // Get valid poses from both limelights and average them if available
    double[] leftPose = getValidTargetSpacePose(LIMELIGHTLEFT);
    double[] rightPose = getValidTargetSpacePose(LIMELIGHTRIGHT);
    double[] pose = averageValidPoses(leftPose, rightPose); // null if neither valid

    boolean leftHas = LimelightHelpers.getTV(LIMELIGHTLEFT);
    boolean rightHas = LimelightHelpers.getTV(LIMELIGHTRIGHT);
    boolean neitherSees = !leftHas && !rightHas;

    boolean isCurrentlyAligned = rotController.atSetpoint()
        && yController.atSetpoint()
        && xController.atSetpoint();

    if (isCurrentlyAligned
        && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME)
        && !alignmentCompleted) {
      alignmentCompleted = true;
      System.out.println("ALIGNMENT ACHIEVED: Starting shooter for "
          + SHOOTER_RUN_TIME + "s");
    }

    // Shooter sequence
    if (alignmentCompleted && !shooterActivated && elevator != null) {
      shooterTimer.restart();
      elevator.setShooterSpeed(-1.0); // run shooter (reverse/full)
      shooterActivated = true;
      System.out.println("SHOOTER STARTED: Continuous for " + SHOOTER_RUN_TIME + "s");
    }

    if (shooterActivated && elevator != null) {
      if (shooterTimer.get() < SHOOTER_RUN_TIME) {
        // keep issuing command while running
        elevator.setShooterSpeed(-1.0);
      } else {
        elevator.setShooterSpeed(0.0);
        System.out.println("SHOOTER COMPLETE: Stopped after " + SHOOTER_RUN_TIME + "s");
      }
      // Hold robot steady during shot
      drivebase.drive(new Translation2d(), 0, false);
     // putDebug(pose, leftPose, rightPose, isCurrentlyAligned, neitherSees);
      return;
    }

    if (pose != null) {
      final double DISTANCE = Math.abs(pose[2]);

      final double minSpeed = 0.75;
      final double maxSpeed = 2.0;
      final double dMin = 0.54;
      final double dMax = 3.0;
    
      // map DISTANCE in [dMin, dMax] -> [minSpeed, maxSpeed], clamp outside range
      double t = (DISTANCE - dMin) / (dMax - dMin);
      if (t < 0) t = 0;
      if (t > 1) t = 1;
    
      // Always positive speed (per your note)
      double xSpeed =( minSpeed + t * (maxSpeed - minSpeed));
      if((Math.abs(yController.getError()) > .5) 
        || Math.abs(rotController.getError())>20)
      {
       // xSpeed*=.25;
      }
    //  System.out.println(yController.getError()+","+rotController.getError());
    //.75
      
      // (leftYSupplier != null) ? leftYSupplier.getAsDouble()
      //                    : xController.calculate(pose[2]);

      double ySpeed = -yController.calculate(pose[0]);
      if(ySpeed<-1) ySpeed= -1;
      if(ySpeed>1) ySpeed=1;
      double rotValue = -rotController.calculate(pose[4]);
      if(rotValue>1)rotValue=1;
      if(rotValue<-1)rotValue=-1;
      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!isCurrentlyAligned) {
        stopTimer.reset();
      }

      // Seeing tags → reset "no target" timer
      dontSeeTagTimer.reset();
    } else {
      // No valid readings from either LL this frame
      drivebase.drive(new Translation2d(), 0, false);

      if (alignmentCompleted && !shooterActivated) {
        alignmentCompleted = false;
        System.out.println("ALIGNMENT LOST: No valid LL poses, resetting completion");
      }
    }

 //   putDebug(pose, leftPose, rightPose, isCurrentlyAligned, neitherSees);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);

    if (elevator != null && shooterActivated) {
      elevator.setShooterSpeed(0.0);
      System.out.println("ALIGNMENT END: Stopping shooter");
    }

    System.out.println("Reef alignment ended: " + (interrupted ? "Interrupted" : "Finished"));
  }

  @Override
  public boolean isFinished() {
    if (driverOverride) return true;

    boolean leftHas = LimelightHelpers.getTV(LIMELIGHTLEFT);
    boolean rightHas = LimelightHelpers.getTV(LIMELIGHTRIGHT);
    boolean neitherSees = !leftHas && !rightHas;

    // With shooter: finish after the shot completes
    if (elevator != null) {
      if (alignmentCompleted && shooterActivated && shooterTimer.get() >= SHOOTER_RUN_TIME) {
        System.out.println("ALIGNREEFREL: Aligned + Shot");
        return true;
      }
      return false;
    }
    return false;

    // Without shooter: finish when aligned and validated
    /*boolean isAligned = rotController.atSetpoint()
        && yController.atSetpoint()
        && xController.atSetpoint();

    return isAligned && stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME);
    */
  }

  // ---------- Helpers ----------

  /** Returns pose array if tv is true and array not all zeros; else null. */
  private double[] getValidTargetSpacePose(String limelightName) {
    if (!LimelightHelpers.getTV(limelightName)) return null;
    double[] pose = LimelightHelpers.getBotPose_TargetSpace(limelightName);
    if (pose == null || pose.length == 0) return null;
    if (isAllZeros(pose)) return null;
    return pose;
  }

  /** Averages two valid pose arrays element-wise. Returns one if only one is valid; null if none. */
  private double[] averageValidPoses(double[] a, double[] b) {
    if (a == null && b == null) return null;
    if (a != null && b == null) return Arrays.copyOf(a, a.length);
    if (a == null && b != null) return Arrays.copyOf(b, b.length);

    int n = Math.min(a.length, b.length); // safety
    double[] out = new double[n];
    for (int i = 0; i < n; i++) {
      out[i] = (a[i] + b[i]) / 2.0;
    }
    return out;
  }

  private boolean isAllZeros(double[] arr) {
    // tolerate tiny noise
    final double eps = 1e-9;
    for (double v : arr) {
      if (Math.abs(v) > eps) return false;
    }
    return true;
  }

  private boolean checkDriverOverride() {
    if (leftYSupplier == null || leftXSupplier == null || rightXSupplier == null) return false;

    // Your version only used rightX for override; keeping that behavior:
    double side = Math.abs(leftXSupplier.getAsDouble());
    double forward = Math.abs(leftYSupplier.getAsDouble());

    double rx = Math.abs(rightXSupplier.getAsDouble());
    boolean override = rx > DRIVER_INPUT_THRESHOLD||side>DRIVER_INPUT_THRESHOLD||forward>DRIVER_INPUT_THRESHOLD;

    return override;
  }

  private void putDebug(double[] avg, double[] left, double[] right,
                        boolean isCurrentlyAligned, boolean neitherSees) {
   
        stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME);

    if (avg != null && avg.length >= 5) {
     
    } else {
    }
  }
}
