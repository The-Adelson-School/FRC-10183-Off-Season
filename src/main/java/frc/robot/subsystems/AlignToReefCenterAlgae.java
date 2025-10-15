package frc.robot.commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Aligns to the reef center (algae offset). If we see one of:
 *  - {6,10,8,17,19,21} -> Algae Position A (low), start algae kicker
 *  - {20,18,22,11,9,7} -> Algae Position B (high), start algae kicker
 * Cancels if driver moves sticks past threshold.
 */
public class AlignToReefCenterAlgae extends Command {

  // Cameras
  private static final String LIMELIGHT_LEFT  = VisionConstants.LEFT_LIMELIGHT_NAME;
  private static final String LIMELIGHT_RIGHT = VisionConstants.RIGHT_LIMELIGHT_NAME;

  // Tag groups
  private static final int[] TAGS_LOW  = {6, 10, 8, 17, 19, 21};
  private static final int[] TAGS_HIGH = {20, 18, 22, 11, 9, 7};

  // Controls
  private final SwerveSubsystem drivebase;
  private final ElevatorSubsystem elevator;
  private final DoubleSupplier leftYSupplier;
  private final DoubleSupplier leftXSupplier;
  private final DoubleSupplier rightXSupplier;

  // PID for center alignment (use your constants)
  private final PIDController yController =
      new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
  private final PIDController rotController =
      new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0);

  // Timers
  private final Timer validPoseTimer = new Timer();

  // State
  private boolean driverOverride = false;
  private boolean algaePresetChosen = false; // chose A/B once
  private static final double DRIVER_INPUT_THRESHOLD = 0.10;

  public AlignToReefCenterAlgae(
      SwerveSubsystem drivebase,
      DoubleSupplier leftY,
      DoubleSupplier leftX,
      DoubleSupplier rightX,
      ElevatorSubsystem elevator) {

    this.drivebase = drivebase;
    this.leftYSupplier = leftY;
    this.leftXSupplier = leftX;
    this.rightXSupplier = rightX;
    this.elevator = elevator;

    // Center setpoints: yaw 0°, Y offset for algae
    rotController.setSetpoint(0.0);
    rotController.setTolerance(1.0);
    yController.setSetpoint(Math.abs(VisionConstants.Y_OFFSET_ALGAE_KICKER));
    yController.setTolerance(2.0);

    addRequirements(drivebase);
    if (elevator != null) addRequirements(elevator);
  }

  @Override
  public void initialize() {
    driverOverride = false;
    algaePresetChosen = false;
    validPoseTimer.restart();
  }

  @Override
  public void execute() {
    // Cancel if driver moves the sticks
    if (checkDriverOverride()) {
      driverOverride = true;
      drivebase.drive(new Translation2d(), 0.0, false);
      return;
    }

    // Choose algae preset once (A/B) based on any visible tag
    if (!algaePresetChosen && elevator != null) {
      Integer fid = getAnyVisibleFiducial();
      if (fid != null) {
        if (contains(TAGS_LOW, fid)) {
          elevator.goToAlgaePositionA();
          algaePresetChosen = true;
        } else if (contains(TAGS_HIGH, fid)) {
          elevator.goToAlgaePositionB();
          algaePresetChosen = true;
        }
      }
    }

    // Get averaged target-space pose (Z = forward distance, X = left/right, yaw at index 4)
    double[] pose = averageValidPoses(
        getValidTargetSpacePose(LIMELIGHT_LEFT),
        getValidTargetSpacePose(LIMELIGHT_RIGHT));

    if (pose != null) {
      // Distance-based forward approach (constant positive, scaled by distance)
      final double distance = Math.abs(pose[2]);
      double xSpeed = mapDistanceToSpeed(distance);

      // Slow approach if large Y or yaw error
      if (Math.abs(yController.getError()) > 0.5 || Math.abs(rotController.getError()) > 20.0) {
        xSpeed *= 0.25;
      }

      double ySpeed = clamp(-yController.calculate(pose[0]), -1, 1);
      double rot    = clamp(-rotController.calculate(pose[4]), -1, 1);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rot, false);
      validPoseTimer.reset();
    } else {
      // No valid pose this tick -> hold still briefly
      drivebase.drive(new Translation2d(), 0.0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0.0, false);
  }

  @Override
  public boolean isFinished() {
    // Ends if driver overrides (sticks moved)
    return driverOverride;
  }

  // ----------------- Helpers -----------------

  private boolean checkDriverOverride() {
    if (leftYSupplier == null || leftXSupplier == null || rightXSupplier == null) return false;
    double side = Math.abs(leftXSupplier.getAsDouble());
    double fwd  = Math.abs(leftYSupplier.getAsDouble());
    double rx   = Math.abs(rightXSupplier.getAsDouble());
    return (rx > DRIVER_INPUT_THRESHOLD) || (side > DRIVER_INPUT_THRESHOLD) || (fwd > DRIVER_INPUT_THRESHOLD);
  }

  private static boolean contains(int[] arr, int v) {
    for (int x : arr) if (x == v) return true;
    return false;
  }

  /** Returns pose array if tv is true and array not all zeros; else null. */
  private static double[] getValidTargetSpacePose(String limelightName) {
    if (!LimelightHelpers.getTV(limelightName)) return null;
    double[] pose = LimelightHelpers.getBotPose_TargetSpace(limelightName);
    if (pose == null || pose.length == 0) return null;
    for (double d : pose) if (Math.abs(d) > 1e-9) return pose;
    return null;
  }

  /** Averages two valid pose arrays element-wise. Returns one if only one valid; null if none. */
  private static double[] averageValidPoses(double[] a, double[] b) {
    if (a == null && b == null) return null;
    if (a != null && b == null) return Arrays.copyOf(a, a.length);
    if (a == null && b != null) return Arrays.copyOf(b, b.length);
    int n = Math.min(a.length, b.length);
    double[] out = new double[n];
    for (int i = 0; i < n; i++) out[i] = (a[i] + b[i]) / 2.0;
    return out;
  }

  /** Returns the primary seen fiducial id (if any) from right, else left. */
  private static Integer getAnyVisibleFiducial() {
    if (LimelightHelpers.getTV(LIMELIGHT_RIGHT)) {
      int id = (int) Math.round(LimelightHelpers.getFiducialID(LIMELIGHT_RIGHT));
      return id > 0 ? id : null;
    }
    if (LimelightHelpers.getTV(LIMELIGHT_LEFT)) {
      int id = (int) Math.round(LimelightHelpers.getFiducialID(LIMELIGHT_LEFT));
      return id > 0 ? id : null;
    }
    return null;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private static double mapDistanceToSpeed(double d) {
    final double minSpeed = 0.75, maxSpeed = 2.0, dMin = 0.54, dMax = 3.0;
    double t = (d - dMin) / (dMax - dMin);
    t = Math.max(0, Math.min(1, t));
    return minSpeed + t * (maxSpeed - minSpeed);
  }
}
