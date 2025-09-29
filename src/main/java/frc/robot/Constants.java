// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(18.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants{

    public static final double MOTOR_ROTATIONS_PER_ELEVATOR_ROTATION = 4.0; // 
    public static final double MAX_EXTENTION_INCHES = 34.5; 
    public static final double INCHES_PER_ROTATION = 1.375;
    
 
    public static final int COUNTS_PER_ROTATION = (int)(2048 * MOTOR_ROTATIONS_PER_ELEVATOR_ROTATION); // 2048 * 4 = 8192
    public static final double COUNTS_PER_INCH = COUNTS_PER_ROTATION/INCHES_PER_ROTATION;

 
    public static final int STOWED_LEVEL = (int)(1.75 * COUNTS_PER_INCH);     
    public static final int LEVEL_ONE = (int)(4 * COUNTS_PER_INCH);            
    public static final int LEVEL_TWO = (int)(22.275 * COUNTS_PER_INCH);
    // Removed LEVEL_THREE since there are only 3 stages (0, 1, 2)


    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 90.0;   
    public static final double MOTION_MAGIC_ACCELERATION = 1800.0;    
    
    public static final double MOTION_MAGIC_KP = 15.0;     
    public static final double MOTION_MAGIC_KI = 0.1;     
    public static final double MOTION_MAGIC_KD = 0.03;    
    public static final double MOTION_MAGIC_KV = 0.2;     
    public static final double MOTION_MAGIC_KS = 0.24;    
    public static final double MOTION_MAGIC_KA = MOTION_MAGIC_CRUISE_VELOCITY / MOTION_MAGIC_ACCELERATION;

    public static final double PEAK_FORWARD_VOLTAGE = 16.0;    
    public static final double PEAK_REVERSE_VOLTAGE = -10.0;   
    
   
    public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 60.0;      
    public static final boolean ELEVATOR_SUPPLY_LIMIT_ENABLE = true;      
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 120.0;      
    public static final boolean ELEVATOR_STATOR_LIMIT_ENABLE = true;      
    
    // Position tolerance for "at target" detection
    public static final double ELEVATOR_POSITION_TOLERANCE = 500;  

    //Intake Speeds
    public static final double INTAKE_IN = 1;
    public static final double INTAKE_STOP = 0.0;
    public static final double INTAKE_OUT = -1;

    public static final double INTAKE_SPEED_MS = 450; //milliseconds to run intake motor

    public static final double INTAKE_P = 0.1; 
    public static final double INTAKE_I = 0.0;
    public static final double INTAKE_D = 0.01;

    public static final double INTAKE_POSITION_TOLERANCE = 0.5;
    public static final double INTAKE_ROTATION_DISTANCE = 1.0;
    
    // Shooter Constants (similar to intake)
    public static final double SHOOTER_ON = 1;         
    public static final double SHOOTER_STOP = 0.0;     
    
    // Current threshold for resistance detection (in amps) - ONLY SHOOTER
    public static final double SHOOTER_CURRENT_THRESHOLD = 25.0;  
    
    // Time threshold for sustained current (in seconds)
    public static final double CURRENT_DETECTION_TIME = 0;      
    
    // Shooter timing for automated sequences
    public static final double SHOOTER_AUTO_RUN_TIME = 0.5;      // Time in seconds to run shooter in automated sequences
  }

  public static class VisionConstants{
    public static final double X_REEF_ALIGNMENT_P = 2;
    public static final double Y_REEF_ALIGNMENT_P = 2;
    public static final double ROT_REEF_ALIGNMENT_P = 0.1;
  
    // Original limelight constants
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.2;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 3;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.53;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 2.5;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.12;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 3;

    // Second limelight constants (opposite side)
    public static final double ROT_SETPOINT_REEF_ALIGNMENT_2 = 0.2;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT_2 = 3;
    public static final double X_SETPOINT_REEF_ALIGNMENT_2 = -0.53;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT_2 = 2.5;
    public static final double Y_SETPOINT_REEF_ALIGNMENT_2 = 0.12;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT_2 = 3;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 1;

    // Limelight Names
    public static final String RIGHT_LIMELIGHT_NAME = "limelight-right";
    public static final String LEFT_LIMELIGHT_NAME = "limelight-left";
    
    // Transform configuration is handled in the Limelight interface, not in code
    // Remove Transform3d constants since they're configured externally
    
    // Vision measurement acceptance thresholds
    public static final double MAX_POSE_AMBIGUITY = 0.3;              // Max ambiguity to accept pose
    public static final double MAX_TAG_DISTANCE = 8.0;                // Max distance in meters
    public static final int MIN_TAG_COUNT = 2;                        // Minimum tags for high confidence
    public static final double SINGLE_TAG_MAX_DISTANCE = 3.0;         // Max distance for single tag
    public static final double MAX_Z_ERROR = 0.5;                     // Max Z-axis error in meters (pose height validation)
    public static final double MAX_AMBIGUITY = 0.2;                   // Max individual tag ambiguity threshold

    // Confidence calculation parameters
    public static final double BASE_CONFIDENCE_SINGLE_TAG = 0.8;      // Base confidence for single tag (meters)
    public static final double BASE_CONFIDENCE_TWO_TAGS = 0.3;        // Base confidence for two tags (meters)
    public static final double BASE_CONFIDENCE_MULTI_TAGS = 0.2;      // Base confidence for 3+ tags (meters)
    public static final double MIN_CONFIDENCE = 0.1;                  // Minimum confidence allowed (meters)
    
    // Distance factor tuning
    public static final double DISTANCE_WEIGHT = 0.3;                 // How much distance affects confidence
    public static final double DISTANCE_SCALE = 5.0;                  // Distance scaling factor
    
    // Tag area factor tuning
    public static final double AREA_WEIGHT = 0.2;                     // How much tag area affects confidence
    public static final double AREA_SCALE = 2.0;                      // Tag area scaling factor
    
    // Standard deviation multipliers for pose estimator
    public static final double XY_STD_DEV_FACTOR = 1.0;               // Multiplier for X and Y standard deviations
    public static final double ROTATION_STD_DEV_FACTOR = 2.0;         // Multiplier for rotation standard deviation (less confident)
    
    // Pipeline configuration
    public static final int APRILTAG_PIPELINE_INDEX = 0;              // Pipeline id for AprilTag 
    
    // MegaTag2 robot orientation update parameters
    public static final boolean ENABLE_MEGATAG2 = true;               // Enable MegaTag2 mode
    public static final double PITCH_DEGREES = 0.0;                   // Robot pitch should be 0
    public static final double ROLL_DEGREES = 0.0;                    // Robot roll should be 0
    public static final double PITCH_RATE_DEG_PER_SEC = 0.0;          // Robot pitch rate should be 0
    public static final double ROLL_RATE_DEG_PER_SEC = 0.0;           // Robot roll rate should be 0
  }

  public static class FieldMovementConstants {
    // X Translation PID constants - tune these for smooth X movement
    public static final double X_TRANSLATION_P = 2.0;      // important
    public static final double X_TRANSLATION_I = 0.0;      // no
    public static final double X_TRANSLATION_D = 0.1;      // no but maybe
    
    // Y Translation PID constants - tune these for smooth Y movement  
    public static final double Y_TRANSLATION_P = 2.0;      // important
    public static final double Y_TRANSLATION_I = 0.0;      // Ino
    public static final double Y_TRANSLATION_D = 0.1;      // no but maybe
    
    // Rotation PID constants - tune these for smooth heading control
    public static final double ROTATION_P = 3.0;           // imrpotant
    public static final double ROTATION_I = 0.0;           // no
    public static final double ROTATION_D = 0.0;           // no byt maybe
    
    // Movement tolerances
    public static final double DEFAULT_POSITION_TOLERANCE = 0.1;  // meters
    public static final double DEFAULT_ROTATION_TOLERANCE = 2.0;  // degrees
    
    // Maximum speeds during auto movement
    public static final double MAX_AUTO_TRANSLATION_SPEED = 3.0;  // m/s
    public static final double MAX_AUTO_ROTATION_SPEED = Math.PI; // rad/s
    
    // Position offsets from AprilTags
    public static final double POSITION_DISTANCE_FROM_TAG = 1.5;      // Distance back from tag for all positions (meters)
    public static final double LEFT_POSITION_OFFSET = 0.6;           // How far left of tag center for LEFT positions (meters)
    public static final double RIGHT_POSITION_OFFSET = 0.6;          // How far right of tag center for RIGHT positions (meters)
  }
}
