package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants
{
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13;
  public static final double MAX_SPEED  = Units.feetToMeters(18.5);

  public static final class DrivebaseConstants
  {
    public static final double WHEEL_LOCK_TIME = 10;
  }

  public static class OperatorConstants
  {
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants{
    public static final double MOTOR_ROTATIONS_PER_ELEVATOR_ROTATION = 4.0;
    public static final double MAX_EXTENTION_INCHES = 34.5; 
    public static final double INCHES_PER_ROTATION = 1.375;
    
    public static final int COUNTS_PER_ROTATION = (int)(2048 * MOTOR_ROTATIONS_PER_ELEVATOR_ROTATION);
    public static final double COUNTS_PER_INCH = COUNTS_PER_ROTATION/INCHES_PER_ROTATION;

    public static final int STOWED_LEVEL = (int)(0 * COUNTS_PER_INCH);     
    public static final int LEVEL_ONE = (int)(23 * COUNTS_PER_INCH);            
    public static final int LEVEL_TWO = (int)(37.7 * COUNTS_PER_INCH);
    public static final int LEVEL_THREE = (int)(50.0 * COUNTS_PER_INCH); // NEW: Stage 3 height
    
    // NEW: Algae kicker specific elevator positions
    public static final int ALGAE_POSITION_A = (int)(12.0 * COUNTS_PER_INCH); // Height for tags 17,11,7,21,9,19
    public static final int ALGAE_POSITION_B = (int)(23.0 * COUNTS_PER_INCH); // Height for tags 18,7,22,6,8,20

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 90.0;   
    public static final double MOTION_MAGIC_ACCELERATION = 1800.0;    
    
    public static final double MOTION_MAGIC_KP = 15.0;     
    public static final double MOTION_MAGIC_KI = 0.1;     
    public static final double MOTION_MAGIC_KD = 0.03;    
    public static final double MOTION_MAGIC_KV = 0.2;     
    public static final double MOTION_MAGIC_KS = 0.24;    
    public static final double MOTION_MAGIC_KA = MOTION_MAGIC_CRUISE_VELOCITY / MOTION_MAGIC_ACCELERATION;
    public static final double PEAK_FORWARD_VOLTAGE = 15.5;    
    public static final double PEAK_REVERSE_VOLTAGE = -3.5;  
    public static final double MOTION_MAGIC_KG = 1.0;
    
    public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 60.0;      
    public static final boolean ELEVATOR_SUPPLY_LIMIT_ENABLE = true;      
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 120.0;      
    public static final boolean ELEVATOR_STATOR_LIMIT_ENABLE = true;      
    
    public static final double ELEVATOR_POSITION_TOLERANCE = 500;

    // Intake only moves one direction
    public static final double INTAKE_OUT = -1;
    public static final double INTAKE_STOP = 0.0;

    public static final double INTAKE_SPEED_MS = 450;
    public static final double INTAKE_POSITION_TOLERANCE = 0.5;
    public static final double INTAKE_ROTATION_DISTANCE = 1.0;
    
    public static final double SHOOTER_ON = -1;         // Reversed direction
    public static final double SHOOTER_STOP = 0.0;     
    
    // Resistance detection - SHOOTER ONLY
    public static final double SHOOTER_CURRENT_THRESHOLD = 17;
    public static final double CURRENT_DETECTION_TIME = 0.5;
    // Following is for Shooting not intaking
    public static final double SHOOTER_AUTO_RUN_TIME = 2;

    // NEW: Algae kicker motor constants
    public static final double ALGAE_KICKER_ON = -1.0;   // Full power forward
    public static final double ALGAE_KICKER_STOP = 0.0; // Stop motor

    // COMMENTED OUT: Cage Motor Constants
    /*
    public static final int CAGE_MOTOR_ID = 21;
    public static final int CAGE_CLIMB_ID = 22;
    
    // Cage Motor Resistance Detection
    public static final double CAGE_CURRENT_THRESHOLD = 20.0; // Amps - changeable
    public static final double CAGE_RESISTANCE_TIME = 0.5; // Seconds - changeable
    
    // Cage Motor Motion Magic Settings
    public static final double CAGE_ROTATION_DEGREES = 45.0; // Degrees - changeable
    public static final double CAGE_GEAR_RATIO = 45.0; // 45:1 gear ratio
    public static final double CAGE_MOTION_MAGIC_CRUISE_VELOCITY = 2.0; // Rotations per second - changeable
    public static final double CAGE_MOTION_MAGIC_ACCELERATION = 4.0; // Rotations per second squared - changeable
    
    // Cage Motor PID Constants (changeable)
    public static final double CAGE_MOTION_MAGIC_KP = 10.0;
    public static final double CAGE_MOTION_MAGIC_KI = 0.0;
    public static final double CAGE_MOTION_MAGIC_KD = 0.0;
    public static final double CAGE_MOTION_MAGIC_KV = 0.12;
    public static final double CAGE_MOTION_MAGIC_KS = 0.25;
    public static final double CAGE_MOTION_MAGIC_KA = 0.0;
    public static final double CAGE_MOTION_MAGIC_KG = 0.0;
    
    // Cage Motor Speeds
    public static final double CAGE_SPIN_SPEED = 1.0; // Full speed - changeable
    public static final double CAGE_STOP = 0.0;
    
    // Cage Motor Current Limits
    public static final double CAGE_SUPPLY_CURRENT_LIMIT = 30.0; // Amps
    public static final double CAGE_STATOR_CURRENT_LIMIT = 60.0; // Amps
    */

  }

  public static class VisionConstants{
    public static final double X_REEF_ALIGNMENT_P = 3;
    public static final double Y_REEF_ALIGNMENT_P = 3.5;
    public static final double ROT_REEF_ALIGNMENT_P = 0.1;
  
    // RIGHT CAMERA SETPOINTS (limelight-right)
    public static final double ROT_SETPOINT_REEF_ALIGNMENT_RIGHT = 0;//-13;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT_RIGHT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT_RIGHT = -0.2;
    public static final double X_TOLERANCE_REEF_ALIGNMENT_RIGHT = 1;
    public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHT = .43; 
    public static final double Y_TOLERANCE_REEF_ALIGNMENT_RIGHT = 1;
    
    // LEFT CAMERA SETPOINTS (limelight-left)  
    public static final double ROT_SETPOINT_REEF_ALIGNMENT_LEFT = 0;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT_LEFT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT_LEFT = 0.2;
    public static final double X_TOLERANCE_REEF_ALIGNMENT_LEFT = 1;
    public static final double Y_SETPOINT_REEF_ALIGNMENT_LEFT = .1; // Base Y setpoint for left camera
    public static final double Y_TOLERANCE_REEF_ALIGNMENT_LEFT = 1;
    
    // SPECIAL CROSS-CAMERA OFFSET: Right alignment using left camera
    public static final double Y_OFFSET_RIGHT_ALIGN_ON_LEFT_CAMERA = 0.2; // Additional offset when aligning right with left camera
    
    // NEW: Algae kicker specific alignment offset
    public static final double Y_OFFSET_ALGAE_KICKER =.3; // Y offset for algae alignment (0.0 = center, adjust as needed)

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 1;

    public static final String RIGHT_LIMELIGHT_NAME = "limelight-right";
    public static final String LEFT_LIMELIGHT_NAME = "limelight-left";
    
    public static final double MAX_POSE_AMBIGUITY = 0.3;
    public static final double MAX_TAG_DISTANCE = 8.0;
    public static final int MIN_TAG_COUNT = 2;
    public static final double SINGLE_TAG_MAX_DISTANCE = 3.0;
    public static final double MAX_Z_ERROR = 0.5;
    public static final double MAX_AMBIGUITY = 0.2;

    public static final double BASE_CONFIDENCE_SINGLE_TAG = 0.8;
    public static final double BASE_CONFIDENCE_TWO_TAGS = 0.3;
    public static final double BASE_CONFIDENCE_MULTI_TAGS = 0.2;
    public static final double MIN_CONFIDENCE = 0.1;
    
    public static final double DISTANCE_WEIGHT = 0.3;
    public static final double DISTANCE_SCALE = 5.0;
    public static final double AREA_WEIGHT = 0.2;
    public static final double AREA_SCALE = 2.0;
    
    public static final double XY_STD_DEV_FACTOR = 1.0;
    public static final double ROTATION_STD_DEV_FACTOR = 2.0;
    
    public static final int APRILTAG_PIPELINE_INDEX = 0;
    
    public static final boolean ENABLE_MEGATAG2 = true;
    public static final double PITCH_DEGREES = 0.0;
    public static final double ROLL_DEGREES = 0.0;
    public static final double PITCH_RATE_DEG_PER_SEC = 0.0;
    public static final double ROLL_RATE_DEG_PER_SEC = 0.0;
  }

  public static class FieldMovementConstants {
    public static final double X_TRANSLATION_P = 0.1;
    public static final double X_TRANSLATION_I = 0.0;
    public static final double X_TRANSLATION_D = 0;
    
    public static final double Y_TRANSLATION_P = 0.1;
    public static final double Y_TRANSLATION_I = 0.0;
    public static final double Y_TRANSLATION_D = 0;
    
    public static final double ROTATION_P = 0.2;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    
    public static final double DEFAULT_POSITION_TOLERANCE = 0.1;
    public static final double DEFAULT_ROTATION_TOLERANCE = 2.0;
    
    public static final double MAX_AUTO_TRANSLATION_SPEED = 1.5;
    public static final double MAX_AUTO_ROTATION_SPEED = Math.PI / 2;
    
    public static final double POSITION_DISTANCE_FROM_TAG = 0;
    public static final double LEFT_POSITION_OFFSET = 0;
    public static final double RIGHT_POSITION_OFFSET = 0;
  }
}
