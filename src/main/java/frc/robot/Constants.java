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
    public static final int LEVEL_ONE = (int)(18.5 * COUNTS_PER_INCH);            
    public static final int LEVEL_TWO = (int)(36.5 * COUNTS_PER_INCH);

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 90.0;   
    public static final double MOTION_MAGIC_ACCELERATION = 1800.0;    
    
    public static final double MOTION_MAGIC_KP = 15.0;     
    public static final double MOTION_MAGIC_KI = 0.1;     
    public static final double MOTION_MAGIC_KD = 0.03;    
    public static final double MOTION_MAGIC_KV = 0.2;     
    public static final double MOTION_MAGIC_KS = 0.24;    
    public static final double MOTION_MAGIC_KA = MOTION_MAGIC_CRUISE_VELOCITY / MOTION_MAGIC_ACCELERATION;
    public static final double PEAK_FORWARD_VOLTAGE = 16.0;    
    public static final double PEAK_REVERSE_VOLTAGE = -4.0;  
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
    public static final double SHOOTER_AUTO_RUN_TIME = 0.5;

  }

  public static class VisionConstants{
    public static final double X_REEF_ALIGNMENT_P = 1;
    public static final double Y_REEF_ALIGNMENT_P = 1;
    public static final double ROT_REEF_ALIGNMENT_P = 0.01;
  
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.2;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 3;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.53;
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 2.5;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.12;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 3;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT_2 = 0.2;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT_2 = 3;
    public static final double X_SETPOINT_REEF_ALIGNMENT_2 = -0.53;
    public static final double X_TOLERANCE_REEF_ALIGNMENT_2 = 2.5;
    public static final double Y_SETPOINT_REEF_ALIGNMENT_2 = 0.12;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT_2 = 3;

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
