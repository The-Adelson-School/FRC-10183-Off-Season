package frc.robot.subsystems.elevator; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; 
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {

    TalonFX elevMotorLeader;    // Main elevator motor
    TalonFX elevMotorFollower;  // Follower elevator motor
    TalonFX intakeMotor;
    TalonFX shooterMotor;       // Shooter motor
    
    // Motion Magic control request
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    
    // Current detection variables - ONLY FOR SHOOTER
    private double shooterHighCurrentStartTime = -1;
    private boolean shooterResistanceDetected = false;
    
    int stage = 0;
    
    public ElevatorSubsystem(int elevLeaderID, int elevFollowerID, int intakeID, int shooterID, int unused2){
        elevMotorLeader = new TalonFX(elevLeaderID);
        elevMotorFollower = new TalonFX(elevFollowerID);
        intakeMotor = new TalonFX(intakeID);
        shooterMotor = new TalonFX(shooterID);  // Initialize shooter motor
        
        // Configure Motion Magic for elevator motors
        configureMotionMagic();
        
        // Configure motors for brake mode
        elevMotorLeader.setNeutralMode(NeutralModeValue.Brake);
        elevMotorFollower.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);  // Shooter brake mode
        
        // Configure follower motor to follow the leader
        elevMotorFollower.setControl(new Follower(elevLeaderID, false)); // false = same direction
        
        // Reset encoder positions to zero
        elevMotorLeader.setPosition(0);
        elevMotorFollower.setPosition(0);
        intakeMotor.setPosition(0);
        shooterMotor.setPosition(0);  // Reset shooter position
        
        System.out.println("Elevator configured with Motion Magic: Leader ID " + elevLeaderID + ", Follower ID " + elevFollowerID + ", Shooter ID " + shooterID);
    }
    
    /**
     * Configure Motion Magic parameters for smooth elevator movement
     */
    private void configureMotionMagic() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motion Magic configuration
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;    // 90 turns/sec
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION;        // 1800 turns/sec²
        
        // PID configuration for Motion Magic
        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kP = ElevatorConstants.MOTION_MAGIC_KP;    // 15
        slot0Configs.kI = ElevatorConstants.MOTION_MAGIC_KI;    // 0.1
        slot0Configs.kD = ElevatorConstants.MOTION_MAGIC_KD;    // 0.03
        slot0Configs.kV = ElevatorConstants.MOTION_MAGIC_KV;    // 0.2
        slot0Configs.kS = ElevatorConstants.MOTION_MAGIC_KS;    // 0.24V for friction
        slot0Configs.kA = ElevatorConstants.MOTION_MAGIC_KA;    // Calculated from constants
        
        // Voltage configuration
        VoltageConfigs voltageConfig = config.Voltage;
        voltageConfig.PeakForwardVoltage = ElevatorConstants.PEAK_FORWARD_VOLTAGE;   // 16V
        voltageConfig.PeakReverseVoltage = ElevatorConstants.PEAK_REVERSE_VOLTAGE;   // -10V
        
        // Current limiting configuration - ELEVATOR ONLY
        CurrentLimitsConfigs currentConfig = config.CurrentLimits;
        currentConfig.withSupplyCurrentLimitEnable(ElevatorConstants.ELEVATOR_SUPPLY_LIMIT_ENABLE);      // true
        currentConfig.withStatorCurrentLimitEnable(ElevatorConstants.ELEVATOR_STATOR_LIMIT_ENABLE);      // true
        currentConfig.withSupplyCurrentLimit(ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT);           // 60A
        currentConfig.withStatorCurrentLimit(ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT);           // 120A
        
        // Apply configuration to both elevator motors
        elevMotorLeader.getConfigurator().apply(config);
        elevMotorFollower.getConfigurator().apply(config);
        
        // Configure intake and shooter motors with minimal config (no current limits)
        TalonFXConfiguration simpleConfig = new TalonFXConfiguration();
        intakeMotor.getConfigurator().apply(simpleConfig);
        shooterMotor.getConfigurator().apply(simpleConfig);
    }

    public int getStage(){
        return stage;
    }

    /**
     * Move elevator to target height using Motion Magic
     * @param elevSetpoint Target position in encoder counts
     */
    private void goToHeight(int elevSetpoint) {
        // Convert from "counts" to rotations for TalonFX
        double targetRotations = (double)elevSetpoint / ElevatorConstants.COUNTS_PER_ROTATION;
        
        // Use Motion Magic to smoothly move to target position
        elevMotorLeader.setControl(motionMagicRequest.withPosition(targetRotations));
        
        // Add telemetry for debugging
        SmartDashboard.putNumber("Elevator Target Rotations", targetRotations);
        SmartDashboard.putNumber("Elevator Target Counts", elevSetpoint);
    }
    
    /**
     * Check if elevator is at target position
     * @param targetCounts Target position in counts
     * @param tolerance Tolerance in counts
     * @return true if within tolerance
     */
    public boolean isElevatorAtTarget(int targetCounts, double tolerance) {
        double currentCounts = elevMotorLeader.getPosition().getValueAsDouble() * ElevatorConstants.COUNTS_PER_ROTATION;
        return Math.abs(currentCounts - targetCounts) <= tolerance;
    }
    
    /**
     * Check if elevator is at target position with default tolerance
     */
    public boolean isElevatorAtTarget(int targetCounts) {
        return isElevatorAtTarget(targetCounts, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE);
    }

    public void setIntakeSpeed(double speed) {
        // Simple direct motor control - intake only stops if SHOOTER resistance is detected
        if (!shooterResistanceDetected) {
            intakeMotor.set(speed);
        } else {
            intakeMotor.set(ElevatorConstants.INTAKE_STOP);
        }
    }
    
    public void setShooterSpeed(double speed) {
        // Don't allow shooter to run if resistance detected
        if (!shooterResistanceDetected) {
            shooterMotor.set(speed);
        } else {
            shooterMotor.set(ElevatorConstants.SHOOTER_STOP);
        }
    }

    public void increaseStage(){
        if(stage < 2){  // Maximum stage is 2 (stages 0, 1, 2)
            stage++;
            engageStage();
        }
    }

    public void decreaseStage(){
        if(stage > 0){
            stage--;
            engageStage();
        }
    }

    public void engageStage() {
        engageStage(stage);
    }
    
    public void engageStage(int targetStage) {
        stage = Math.max(0, Math.min(2, targetStage)); // Clamp to 0-2 range
        SmartDashboard.putNumber("Stage", stage);
        
        // Reset resistance detection when changing stages
        resetResistanceDetection();
        
        if (stage == 0) {
            goToHeight(ElevatorConstants.STOWED_LEVEL);
            // Auto-start intake and shooter when in stage 0
            setIntakeSpeed(ElevatorConstants.INTAKE_IN);
            setShooterSpeed(ElevatorConstants.SHOOTER_ON);
        } else if (stage == 1) {
            goToHeight(ElevatorConstants.LEVEL_ONE);
            // Auto-stop intake and shooter when not in stage 0
            setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
            setShooterSpeed(ElevatorConstants.SHOOTER_STOP);
        } else if (stage == 2) {
            goToHeight(ElevatorConstants.LEVEL_TWO);
            // Auto-stop intake and shooter when not in stage 0
            setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
            setShooterSpeed(ElevatorConstants.SHOOTER_STOP);
        }
    }

    public void defaultCommand() {
        engageStage(); 
    }

    public void autonomousCommand(){
        // REMOVE THREAD.SLEEP - Use command scheduling instead
        stage = 1;
        engageStage();
        
        // Don't use Thread.sleep() - it blocks the entire robot!
        // Instead, this should be handled by command scheduling
        // The caller should use WaitCommand or SequentialCommandGroup
        
        setIntakeSpeed(ElevatorConstants.INTAKE_OUT);
        
        stage = 0;
        engageStage();
        
        // Don't use Thread.sleep() - it blocks the entire robot!
        // The timing should be handled by the command that calls this method
    }
    
    /**
     * Create a proper autonomous command sequence that doesn't block
     */
    public Command createAutonomousCommand() {
        return Commands.sequence(
            // Move to stage 1
            Commands.runOnce(() -> engageStage(1)),
            // Wait for elevator to reach position
            Commands.waitUntil(() -> isElevatorAtTarget(ElevatorConstants.LEVEL_ONE)),
            // Wait 1.5 seconds
            Commands.waitSeconds(1.5),
            // Set intake to output
            Commands.runOnce(() -> setIntakeSpeed(ElevatorConstants.INTAKE_OUT)),
            // Move to stage 0
            Commands.runOnce(() -> engageStage(0)),
            // Wait for elevator to reach position
            Commands.waitUntil(() -> isElevatorAtTarget(ElevatorConstants.STOWED_LEVEL)),
            // Wait another 1.5 seconds
            Commands.waitSeconds(1.5)
        ).withName("ElevatorAutonomous");
    }

    @Override
    public void periodic() {
        // Update current detection first
        updateCurrentDetection();
        
        // Handle automatic shutoff if SHOOTER resistance detected
        if (shooterResistanceDetected) {
            shooterMotor.set(ElevatorConstants.SHOOTER_STOP);
            intakeMotor.set(ElevatorConstants.INTAKE_STOP);  // Intake stops when SHOOTER has resistance
        }
        
        // Add telemetry for debugging - monitor all motors
        SmartDashboard.putNumber("Elevator Leader Position", elevMotorLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Position", elevMotorFollower.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Position", intakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Position", shooterMotor.getPosition().getValueAsDouble());
        
        // Current monitoring
        double leaderCurrent = elevMotorLeader.getStatorCurrent().getValueAsDouble();
        double followerCurrent = elevMotorFollower.getStatorCurrent().getValueAsDouble();
        double intakeCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
        double shooterCurrent = shooterMotor.getStatorCurrent().getValueAsDouble();
        
        SmartDashboard.putNumber("Elevator Leader Current", leaderCurrent);
        SmartDashboard.putNumber("Elevator Follower Current", followerCurrent);
        SmartDashboard.putNumber("Intake Current", intakeCurrent);
        SmartDashboard.putNumber("Shooter Current", shooterCurrent);
        
        // Show combined elevator current and current limit status
        double totalElevatorCurrent = leaderCurrent + followerCurrent;
        SmartDashboard.putNumber("Total Elevator Current", totalElevatorCurrent);
        
        // Current limit warnings - ELEVATOR ONLY
        SmartDashboard.putBoolean("Elevator Leader Current Warning", leaderCurrent > ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT * 0.8);
        SmartDashboard.putBoolean("Elevator Follower Current Warning", followerCurrent > ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT * 0.8);
        
        // Resistance detection status - ONLY SHOOTER
        SmartDashboard.putBoolean("Shooter Resistance Detected", shooterResistanceDetected);
        SmartDashboard.putBoolean("Shooter Above Threshold", shooterCurrent > ElevatorConstants.SHOOTER_CURRENT_THRESHOLD);
        SmartDashboard.putNumber("Intake Current (Info Only)", intakeCurrent);
        
        // Motion Magic status
        SmartDashboard.putNumber("Elevator Leader Velocity", elevMotorLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Error", elevMotorLeader.getClosedLoopError().getValueAsDouble());
        
        // Check if at current stage target
        int currentTarget = getCurrentStageTarget();
        SmartDashboard.putBoolean("Elevator At Target", isElevatorAtTarget(currentTarget));
        
        // Supply current monitoring
        SmartDashboard.putNumber("Elevator Leader Supply Current", elevMotorLeader.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Supply Current", elevMotorFollower.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Supply Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Supply Current", shooterMotor.getSupplyCurrent().getValueAsDouble());
    }
    
    /**
     * Get the target position for the current stage
     */
    private int getCurrentStageTarget() {
        switch (stage) {
            case 0: return ElevatorConstants.STOWED_LEVEL;
            case 1: return ElevatorConstants.LEVEL_ONE;
            case 2: return ElevatorConstants.LEVEL_TWO;
            default: return ElevatorConstants.STOWED_LEVEL;
        }
    }
    
    /**
     * Emergency stop for elevator - stops all motors immediately
     */
    public void emergencyStop() {
        elevMotorLeader.set(0);
        intakeMotor.set(0);
        shooterMotor.set(0);
        resetResistanceDetection();
        System.out.println("ELEVATOR EMERGENCY STOP ACTIVATED");
    }
    
    /**
     * Get the average position of both elevator motors for verification
     */
    public double getAverageElevatorPosition() {
        double leaderPos = elevMotorLeader.getPosition().getValueAsDouble();
        double followerPos = elevMotorFollower.getPosition().getValueAsDouble();
        return (leaderPos + followerPos) / 2.0;
    }
    
    /**
     * Check if elevator motors are synchronized (within tolerance)
     */
    public boolean areMotorsSynchronized() {
        double leaderPos = elevMotorLeader.getPosition().getValueAsDouble();
        double followerPos = elevMotorFollower.getPosition().getValueAsDouble();
        double tolerance = 0.1; // 0.1 rotations tolerance
        return Math.abs(leaderPos - followerPos) < tolerance;
    }
    
    /**
     * Manually set elevator position using Motion Magic (for testing/tuning)
     */
    public void setElevatorPosition(double rotations) {
        elevMotorLeader.setControl(motionMagicRequest.withPosition(rotations));
    }
    
    /**
     * Get current elevator position in rotations
     */
    public double getElevatorPosition() {
        return elevMotorLeader.getPosition().getValueAsDouble();
    }
    
    /**
     * Get current elevator position in counts
     */
    public double getElevatorPositionCounts() {
        return getElevatorPosition() * ElevatorConstants.COUNTS_PER_ROTATION;
    }
    
    /**
     * Command factory for moving to a specific stage and waiting until complete
     */
    public Command moveToStageCommand(int targetStage) {
        return Commands.runOnce(() -> engageStage(targetStage))
            .andThen(Commands.waitUntil(() -> isElevatorAtTarget(getCurrentStageTarget())))
            .withName("MoveToStage" + targetStage);
    }
    
    /**
     * Check for current-based resistance detection - ONLY SHOOTER
     */
    private void updateCurrentDetection() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Check ONLY shooter current - intake current is ignored
        double shooterCurrent = shooterMotor.getStatorCurrent().getValueAsDouble();
        if (shooterCurrent > ElevatorConstants.SHOOTER_CURRENT_THRESHOLD) {
            if (shooterHighCurrentStartTime < 0) {
                shooterHighCurrentStartTime = currentTime;
            } else if (currentTime - shooterHighCurrentStartTime > ElevatorConstants.CURRENT_DETECTION_TIME) {
                if (!shooterResistanceDetected) {
                    System.out.println("SHOOTER RESISTANCE DETECTED - Auto-stopping BOTH shooter and intake");
                    shooterResistanceDetected = true;
                }
            }
        } else {
            shooterHighCurrentStartTime = -1;
        }
    }
    
    /**
     * Reset resistance detection (call when stage changes or manually)
     */
    public void resetResistanceDetection() {
        shooterResistanceDetected = false;
        shooterHighCurrentStartTime = -1;
        System.out.println("Resistance detection reset");
    }
    
    /**
     * Get shooter current for external monitoring
     */
    public double getShooterCurrent() {
        return shooterMotor.getStatorCurrent().getValueAsDouble();
    }
    
    /**
     * Check if shooter resistance is detected
     */
    public boolean isShooterResistanceDetected() {
        return shooterResistanceDetected;
    }
}