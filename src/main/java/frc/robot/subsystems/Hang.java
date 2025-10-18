package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public class Hang extends SubsystemBase {
    private final TalonFX climbMotor;
    private final TalonFX climberIntakeMotor;
    
    // Motor IDs
    private static final int CLIMB_MOTOR_ID = 41;
    private static final int CLIMBER_INTAKE_ID = 42;
    
    // Gear ratio and stage positions
    private static final double GEAR_RATIO = 45.0; 
    private static final double STAGE_0_ROTATIONS = 0.0;
    private static final double STAGE_1_ROTATIONS = -1.5;
    private static final double STAGE_2_ROTATIONS = -9.5;
    

    private static final double PEAK_FORWARD_VOLTAGE = 50;    
    private static final double PEAK_REVERSE_VOLTAGE = -3.5;
    
    private int currentStage = 0;
    
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    
    // Shuffleboard entries
    private final ShuffleboardTab hangTab = Shuffleboard.getTab("Hang");
    private final GenericEntry climbMotorEncoderEntry;
    private final GenericEntry climbMotorRotationsEntry;
    private final GenericEntry currentStageEntry;
    
    public Hang() {
        climbMotor = new TalonFX(CLIMB_MOTOR_ID, "CANivore");
        
        climberIntakeMotor = new TalonFX(CLIMBER_INTAKE_ID, "CANivore");
        
        // Initialize Shuffleboard entries
        climbMotorEncoderEntry = hangTab.add("Climb Motor Encoder", 0.0).getEntry();
        climbMotorRotationsEntry = hangTab.add("Climb Motor Rotations", 0.0).getEntry();
        currentStageEntry = hangTab.add("Current Stage", 0).getEntry();
        
        configureMotors();
        
        setStage(0);
    }
    
    private void configureMotors() {
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        
        MotionMagicConfigs motionMagicConfigs = climbConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 90;
        motionMagicConfigs.MotionMagicAcceleration = 1800;
        
        Slot0Configs slot0Configs = climbConfig.Slot0;
        slot0Configs.kP = 15.0;
        slot0Configs.kI = 0.1;  
        slot0Configs.kD = 0.03;  
        slot0Configs.kV = 0.2; 
        
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.PeakForwardDutyCycle = PEAK_FORWARD_VOLTAGE / 12.0;
        climbConfig.MotorOutput.PeakReverseDutyCycle = PEAK_REVERSE_VOLTAGE / 12.0;
        climbConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        climbMotor.getConfigurator().apply(climbConfig);
        
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        climberIntakeMotor.getConfigurator().apply(intakeConfig);
        
        climbMotor.setPosition(0.0);
        climberIntakeMotor.setPosition(0.0);
        
        System.out.println("Hang subsystem motors configured successfully");
    }
    
  
    public void increaseStage() {
        System.out.println("HANG DEBUG: increaseStage() called - current stage: " + currentStage);
        if (currentStage < 2) {
            currentStage++;
            setStage(currentStage);
            System.out.println("Hang: Increased to stage " + currentStage);
        } else {
            System.out.println("Hang: Already at maximum stage (2), cannot increase");
        }
    }
    

    public void decreaseStage() {
        System.out.println("HANG DEBUG: decreaseStage() called - current stage: " + currentStage);
        if (currentStage > 0) {
            currentStage--;
            setStage(currentStage);
            System.out.println("Hang: Decreased to stage " + currentStage);
        } else {
            System.out.println("Hang: Already at minimum stage (0), cannot decrease");
        }
    }
    
  
    public void cycleStage() {
        System.out.println("HANG DEBUG: cycleStage() called - current stage: " + currentStage);
        currentStage = (currentStage + 1) % 3; // Cycle through 0, 1, 2, then back to 0
        System.out.println("HANG DEBUG: new stage: " + currentStage);
        setStage(currentStage);
        
        System.out.println("Hang: Cycling to stage " + currentStage);
    }
    
  
    public void setStage(int stage) {
        System.out.println("HANG DEBUG: setStage(" + stage + ") called");
        currentStage = Math.max(0, Math.min(2, stage)); // Clamp between 0-2
        
        double targetRotations;
        switch (currentStage) {
            case 0:
                targetRotations = STAGE_0_ROTATIONS;
                stopClimberIntake();
                break;
            case 1:
                targetRotations = STAGE_1_ROTATIONS;
                startClimberIntake();
                break;
            case 2:
                targetRotations = STAGE_2_ROTATIONS;
                stopClimberIntake();
                break;
            default:
                targetRotations = STAGE_0_ROTATIONS;
                stopClimberIntake();
                break;
        }
        
        // Convert rotations to motor rotations (account for gear ratio)
        double motorRotations = targetRotations * GEAR_RATIO;
        
        System.out.println("HANG DEBUG: Sending Motion Magic command - target: " + motorRotations + " motor rotations");
        
        // Send Motion Magic command
        climbMotor.setControl(motionMagicRequest.withPosition(motorRotations));
        
        System.out.println("Hang: Moving to stage " + currentStage + 
                          " (Target: " + targetRotations + " rotations, Motor: " + motorRotations + " rotations)");
    }
    
    /**
     * Manual control of climber intake motor for operator D-pad
     */
    public void setClimberIntakeSpeed(double speed) {
        climberIntakeMotor.set(speed);
        if (speed != 0.0) {
            System.out.println("Hang: Manual climber intake speed set to " + speed);
        }
    }
    
    /**
     * Start climber intake at full power
     */
    private void startClimberIntake() {
        climberIntakeMotor.set(-1.0); // Full power
        System.out.println("Hang: Starting climber intake at full power");
    }
    
    /**
     * Stop climber intake
     */
    private void stopClimberIntake() {
        climberIntakeMotor.set(0.0);
        System.out.println("Hang: Stopping climber intake");
    }
    
    /**
     * Get current stage
     */
    public int getCurrentStage() {
        return currentStage;
    }
    
    /**
     * Check if elevator is at target position
     */
    public boolean isAtTarget() {
        double currentPosition = climbMotor.getPosition().getValueAsDouble();
        double targetPosition;
        
        switch (currentStage) {
            case 0:
                targetPosition = STAGE_0_ROTATIONS * GEAR_RATIO;
                break;
            case 1:
                targetPosition = STAGE_1_ROTATIONS * GEAR_RATIO;
                break;
            case 2:
                targetPosition = STAGE_2_ROTATIONS * GEAR_RATIO;
                break;
            default:
                targetPosition = STAGE_0_ROTATIONS * GEAR_RATIO;
                break;
        }
        
        return Math.abs(currentPosition - targetPosition) < (0.1 * GEAR_RATIO); // Within 0.1 rotation tolerance
    }
    
    /**
     * Get current position in rotations (output shaft)
     */
    public double getCurrentRotations() {
        return climbMotor.getPosition().getValueAsDouble() / GEAR_RATIO;
    }
    
    /**
     * Manual control for testing (use with caution)
     */
    public void setManualPower(double power) {
        climbMotor.set(power);
        if (power != 0.0) {
            System.out.println("Hang: Manual climber motor power set to " + power);
        }
    }
    
    /**
     * Stop all motors
     */
    public void stop() {
        climbMotor.set(0.0);
        climberIntakeMotor.set(0.0);
    }
    
    @Override
    public void periodic() {
        // Add telemetry for debugging
        double currentPos = climbMotor.getPosition().getValueAsDouble();
        double currentRotations = getCurrentRotations();
        
       
        climbMotorEncoderEntry.setDouble(currentPos);
        climbMotorRotationsEntry.setDouble(currentRotations);
        currentStageEntry.setInteger(currentStage);
       
        if (Math.random() < 0.02) {
            System.out.println( 
                             "Motor Rotations=" + String.format("%.2f", currentPos) + 
                             "Real Rotations=" + String.format("%.2f", currentRotations));
                             
        }
    }
}
