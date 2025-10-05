package frc.robot.subsystems.elevator; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public class ElevatorSubsystem extends SubsystemBase {

    TalonFX elevMotorLeader;
    TalonFX elevMotorFollower;
    TalonFX intakeMotor;
    TalonFX shooterMotor;
    
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    
    // Resistance detection - SHOOTER ONLY
    private double shooterHighCurrentStartTime = -1;
    private boolean shooterResistanceDetected = false;
    
    // Average current tracking
    private double avgCurrent = 0.0;
    
    // Manual shooter override
    private boolean manualShooterOverride = false;
    
    int stage = 0;
    
    // Shuffleboard for current monitoring
    private final ShuffleboardTab currentTab = Shuffleboard.getTab("Current Monitoring");
    private final GenericEntry shooterCurrentEntry = currentTab.add("Shooter Current (A)", 0.0).getEntry();
    private final GenericEntry shooterCurrentGraphEntry = currentTab.add("Shooter Current Graph", 0.0)
        .withWidget("Graph")
        .withSize(4, 3)
        .getEntry();
    
    public ElevatorSubsystem(int elevLeaderID, int elevFollowerID, int intakeID, int shooterID, int unused2){
        elevMotorLeader = new TalonFX(elevLeaderID, "CANivore");
        elevMotorFollower = new TalonFX(elevFollowerID, "CANivore");
        intakeMotor = new TalonFX(intakeID, "CANivore");
        shooterMotor = new TalonFX(shooterID, "CANivore");
        
        configureMotionMagic();
        
        elevMotorLeader.setNeutralMode(NeutralModeValue.Brake);
        elevMotorFollower.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Follower motor follows leader
        elevMotorFollower.setControl(new Follower(elevLeaderID, false));
        
        elevMotorLeader.setPosition(0);
        elevMotorFollower.setPosition(0);
        intakeMotor.setPosition(0);
        shooterMotor.setPosition(0);
    }
    
    private void configureMotionMagic() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION;
        
        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kP = ElevatorConstants.MOTION_MAGIC_KP;
        slot0Configs.kI = ElevatorConstants.MOTION_MAGIC_KI;
        slot0Configs.kD = ElevatorConstants.MOTION_MAGIC_KD;
        slot0Configs.kV = ElevatorConstants.MOTION_MAGIC_KV;
        slot0Configs.kS = ElevatorConstants.MOTION_MAGIC_KS;
        slot0Configs.kA = ElevatorConstants.MOTION_MAGIC_KA;
        slot0Configs.kG = ElevatorConstants.MOTION_MAGIC_KG;
        
        VoltageConfigs voltageConfig = config.Voltage;
        voltageConfig.PeakForwardVoltage = ElevatorConstants.PEAK_FORWARD_VOLTAGE;
        voltageConfig.PeakReverseVoltage = ElevatorConstants.PEAK_REVERSE_VOLTAGE;
        
        // Current limits - ELEVATOR ONLY
        CurrentLimitsConfigs currentConfig = config.CurrentLimits;
        currentConfig.withSupplyCurrentLimitEnable(ElevatorConstants.ELEVATOR_SUPPLY_LIMIT_ENABLE);
        currentConfig.withStatorCurrentLimitEnable(ElevatorConstants.ELEVATOR_STATOR_LIMIT_ENABLE);
        currentConfig.withSupplyCurrentLimit(ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT);
        currentConfig.withStatorCurrentLimit(ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT);
        
        elevMotorLeader.getConfigurator().apply(config);
        elevMotorFollower.getConfigurator().apply(config);
        
        // Configuration for intake motor (30A supply, 60A stator)
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeCurrentConfig = intakeConfig.CurrentLimits;
        intakeCurrentConfig.withSupplyCurrentLimitEnable(true);
        intakeCurrentConfig.withStatorCurrentLimitEnable(true);
        intakeCurrentConfig.withSupplyCurrentLimit(30.0);  // 30A supply limit
        intakeCurrentConfig.withStatorCurrentLimit(60.0);  // 60A stator limit
        
        intakeMotor.getConfigurator().apply(intakeConfig);
        
        // Configuration for shooter motor (30A supply, 35A stator) - INCREASED FOR B BUTTON
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs shooterCurrentConfig = shooterConfig.CurrentLimits;
        shooterCurrentConfig.withSupplyCurrentLimitEnable(true);
        shooterCurrentConfig.withStatorCurrentLimitEnable(true);
        shooterCurrentConfig.withSupplyCurrentLimit(30.0);  // 30A supply limit (increased from 8A)
        shooterCurrentConfig.withStatorCurrentLimit(35.0);  // 35A stator limit
        
        shooterMotor.getConfigurator().apply(shooterConfig);
    }

    public int getStage(){
        return stage;
    }

    private void goToHeight(int elevSetpoint) {
        double targetRotations = (double)elevSetpoint / ElevatorConstants.COUNTS_PER_ROTATION;
        elevMotorLeader.setControl(motionMagicRequest.withPosition(targetRotations));
        SmartDashboard.putNumber("Elevator Target Rotations", targetRotations);
        SmartDashboard.putNumber("Elevator Target Counts", elevSetpoint);
    }
    
    public boolean isElevatorAtTarget(int targetCounts, double tolerance) {
        double currentCounts = elevMotorLeader.getPosition().getValueAsDouble() * ElevatorConstants.COUNTS_PER_ROTATION;
        return Math.abs(currentCounts - targetCounts) <= tolerance;
    }
    
    public boolean isElevatorAtTarget(int targetCounts) {
        return isElevatorAtTarget(targetCounts, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE);
    }

    public void setIntakeSpeed(double speed) {
        // In stages 1 and 2, allow manual control
        // In stage 0, this method is overridden by periodic() for automatic control
        if (stage != 0) {
            intakeMotor.set(speed);
        }
        // Stage 0 intake control is handled in periodic() method automatically
    }
    
    public void setShooterSpeed(double speed) {
        // Resistance detection only in stage 0
        if (stage == 0 && shooterResistanceDetected) {
            shooterMotor.set(ElevatorConstants.SHOOTER_STOP);
        } else {
            shooterMotor.set(speed);
        }
    }
    
    /**
     * Set manual shooter override (for A button control)
     */
    public void setManualShooterOverride(boolean override) {
        this.manualShooterOverride = override;
    }

    public void increaseStage(){
        if(stage < 2){
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
        int previousStage = stage;
        stage = Math.max(0, Math.min(2, targetStage));
        SmartDashboard.putNumber("Stage", stage);
        
        // Reset resistance detection when leaving stage 0
        if (previousStage == 0 && stage != 0) {
            resetResistanceDetection();
        }
        
        if (stage == 0) {
            goToHeight(ElevatorConstants.STOWED_LEVEL);
            // Don't manually set intake/shooter here - let periodic() handle it automatically
        } else if (stage == 1) {
            goToHeight(ElevatorConstants.LEVEL_ONE);
            setIntakeSpeed(ElevatorConstants.INTAKE_STOP); // Manual control in stages 1 and 2
            // Shooter controlled by manual override in periodic()
        } else if (stage == 2) {
            goToHeight(ElevatorConstants.LEVEL_TWO);
            setIntakeSpeed(ElevatorConstants.INTAKE_STOP); // Manual control in stages 1 and 2
            // Shooter controlled by manual override in periodic()
        }
    }

    public void defaultCommand() {
        engageStage(); 
    }

    public void autonomousCommand(){
        stage = 1;
        engageStage();
        setIntakeSpeed(ElevatorConstants.INTAKE_OUT);
        stage = 0;
        engageStage();
    }
    
    public Command createAutonomousCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> engageStage(1)),
            Commands.waitUntil(() -> isElevatorAtTarget(ElevatorConstants.LEVEL_ONE)),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> setIntakeSpeed(ElevatorConstants.INTAKE_OUT)),
            Commands.runOnce(() -> engageStage(0)),
            Commands.waitUntil(() -> isElevatorAtTarget(ElevatorConstants.STOWED_LEVEL)),
            Commands.waitSeconds(1.5)
        ).withName("ElevatorAutonomous");
    }

    @Override
    public void periodic() {
        // Shooter control logic based on stage and manual override
        boolean shooterShouldRun = false;
        
        if (stage == 0) {
            // Stage 0: Run continuously unless resistance detected
            shooterShouldRun = true;
        } else {
            // Stages 1 and 2: Only run when manual override is active (B button pressed)
            if (manualShooterOverride) {
                shooterShouldRun = true;
            }
            // Otherwise shooter stays OFF in stages 1 and 2
        }
        
        // Update resistance detection when shooter should be running
        if (shooterShouldRun) {
            updateCurrentDetection();
        } else {
            // Reset resistance detection when shooter shouldn't be running
            shooterResistanceDetected = false;
        }
        
        // Apply resistance detection - motors turn off if resistance detected
        if (shooterShouldRun && !shooterResistanceDetected) {
            // Should run and no resistance - turn motors ON
            shooterMotor.set(ElevatorConstants.SHOOTER_ON);
            if (stage == 0) {
                intakeMotor.set(ElevatorConstants.INTAKE_OUT); // Intake only runs automatically in stage 0
            }
        } else {
            // Either shouldn't run or resistance detected - turn motors OFF
            shooterMotor.set(ElevatorConstants.SHOOTER_STOP);
            if (stage == 0) {
                intakeMotor.set(ElevatorConstants.INTAKE_STOP); // Stop intake when resistance detected in stage 0
            }
        }
        
        SmartDashboard.putNumber("Elevator Leader Position", elevMotorLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Position", elevMotorFollower.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Position", intakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Position", shooterMotor.getPosition().getValueAsDouble());
        
        double leaderCurrent = elevMotorLeader.getStatorCurrent().getValueAsDouble();
        double followerCurrent = elevMotorFollower.getStatorCurrent().getValueAsDouble();
        double intakeCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
        double shooterCurrent = shooterMotor.getStatorCurrent().getValueAsDouble();
        
        // Create shooter supply current variable and calculate average current
        double shooterSupplyCurrent = shooterMotor.getSupplyCurrent().getValueAsDouble();
        avgCurrent = (0.95 * avgCurrent) + (0.05 * shooterSupplyCurrent);
        
        SmartDashboard.putNumber("Shooter Supply Current", shooterSupplyCurrent);
        SmartDashboard.putNumber("Average Current", avgCurrent);
        
        // Update shooter current on Glass for real-time plotting (using actual supply current)
        shooterCurrentEntry.setDouble(shooterSupplyCurrent);
        shooterCurrentGraphEntry.setDouble(shooterSupplyCurrent);
        
        SmartDashboard.putNumber("Elevator Leader Current", leaderCurrent);
        SmartDashboard.putNumber("Elevator Follower Current", followerCurrent);
        SmartDashboard.putNumber("Intake Current", intakeCurrent);
        SmartDashboard.putNumber("Shooter Current", shooterCurrent);
        
        double totalElevatorCurrent = leaderCurrent + followerCurrent;
        SmartDashboard.putNumber("Total Elevator Current", totalElevatorCurrent);
        
        SmartDashboard.putBoolean("Elevator Leader Current Warning", leaderCurrent > ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT * 0.8);
        SmartDashboard.putBoolean("Elevator Follower Current Warning", followerCurrent > ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT * 0.8);
        
        SmartDashboard.putBoolean("Shooter Resistance Detected", shooterResistanceDetected && stage == 0);
        SmartDashboard.putBoolean("Shooter Above Threshold", shooterSupplyCurrent > ElevatorConstants.SHOOTER_CURRENT_THRESHOLD);
        SmartDashboard.putBoolean("Resistance Detection Active", stage == 0);
        
        SmartDashboard.putNumber("Elevator Leader Velocity", elevMotorLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Leader Error", elevMotorLeader.getClosedLoopError().getValueAsDouble());
        
        int currentTarget = getCurrentStageTarget();
        SmartDashboard.putBoolean("Elevator At Target", isElevatorAtTarget(currentTarget));
    }

    private int getCurrentStageTarget() {
        switch (stage) {
            case 0: return ElevatorConstants.STOWED_LEVEL;
            case 1: return ElevatorConstants.LEVEL_ONE;
            case 2: return ElevatorConstants.LEVEL_TWO;
            default: return ElevatorConstants.STOWED_LEVEL;
        }
    }
    
    public void emergencyStop() {
        elevMotorLeader.set(0);
        intakeMotor.set(0);
        shooterMotor.set(0);
        resetResistanceDetection();
    }
    
    public Command moveToStageCommand(int targetStage) {
        return Commands.runOnce(() -> engageStage(targetStage))
            .andThen(Commands.waitUntil(() -> isElevatorAtTarget(getCurrentStageTarget())))
            .withName("MoveToStage" + targetStage);
    }
    
    // Resistance detection - TIMER-BASED WITH PERSISTENCE (NO GRACE PERIOD)
    private void updateCurrentDetection() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Use average current instead of instantaneous current
        double shooterCurrent = avgCurrent;
        
        // ENHANCED DEBUG OUTPUT - always show current values
        SmartDashboard.putNumber("Shooter Supply Current", shooterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Average Current (Used for Detection)", shooterCurrent);
        SmartDashboard.putNumber("Current Threshold", ElevatorConstants.SHOOTER_CURRENT_THRESHOLD);
        SmartDashboard.putBoolean("Current Above Threshold", shooterCurrent > ElevatorConstants.SHOOTER_CURRENT_THRESHOLD);
        SmartDashboard.putNumber("Stage", stage);
        SmartDashboard.putBoolean("In Stage 0", stage == 0);
        
        // Debug console output every 50 loops (~1 second)
        if ((int)(currentTime * 50) % 50 == 0) {
            System.out.println(String.format("DEBUG: Stage=%d, AvgCurrent=%.2fA, Threshold=%.2fA, Above=%b, Detected=%b", 
                stage, shooterCurrent, ElevatorConstants.SHOOTER_CURRENT_THRESHOLD, 
                shooterCurrent > ElevatorConstants.SHOOTER_CURRENT_THRESHOLD, shooterResistanceDetected));
        }
        
        // Only check for resistance if threshold is greater than 0
        if (ElevatorConstants.SHOOTER_CURRENT_THRESHOLD <= 0) {
            SmartDashboard.putString("Resistance Status", "Disabled (Threshold = 0)");
            shooterResistanceDetected = false; // Disabled
            shooterHighCurrentStartTime = -1;
            return;
        }
        
        // Once resistance is detected, it stays detected until manual reset
        if (shooterResistanceDetected) {
            SmartDashboard.putString("Resistance Status", "LOCKED - Resistance Detected (Manual Reset Required)");
            // Don't process further - resistance flag persists until manual reset
            return;
        }
        
        // Timer-based resistance detection - only if not already detected
        if (shooterCurrent > ElevatorConstants.SHOOTER_CURRENT_THRESHOLD) {
            // High current detected
            if (shooterHighCurrentStartTime < 0) {
                // First time seeing high current - start the timer
                shooterHighCurrentStartTime = currentTime;
                SmartDashboard.putString("Resistance Status", "High Current - Timer Started");
                System.out.println("HIGH CURRENT TIMER STARTED: " + shooterCurrent + "A (Threshold: " + ElevatorConstants.SHOOTER_CURRENT_THRESHOLD + "A)");
            } else {
                // High current ongoing - check if timer has elapsed
                double highCurrentDuration = currentTime - shooterHighCurrentStartTime;
                SmartDashboard.putString("Resistance Status", 
                    String.format("High Current - Duration: %.3fs/%.3fs", 
                    highCurrentDuration, ElevatorConstants.CURRENT_DETECTION_TIME));
                SmartDashboard.putNumber("High Current Duration", highCurrentDuration);
                
                System.out.println(String.format("HIGH CURRENT ONGOING: %.3fs/%.3fs", 
                    highCurrentDuration, ElevatorConstants.CURRENT_DETECTION_TIME));
                
                if (highCurrentDuration >= ElevatorConstants.CURRENT_DETECTION_TIME) {
                    // Timer elapsed - resistance is confirmed
                    shooterResistanceDetected = true;
                    System.out.println("!!!!! RESISTANCE DETECTED - Motors LOCKED OFF !!!!!");
                    System.out.println("Average Current: " + shooterCurrent + "A, Threshold: " + ElevatorConstants.SHOOTER_CURRENT_THRESHOLD + "A");
                    System.out.println("High current duration: " + highCurrentDuration + " seconds");
                    System.out.println("Motors will remain OFF until manual reset (Start button)");
                }
            }
        } else {
            // Normal current - reset the high current timer but DON'T clear resistance flag
            if (shooterHighCurrentStartTime >= 0) {
                System.out.println("CURRENT DROPPED TO NORMAL: " + shooterCurrent + "A - Resetting high current timer");
                shooterHighCurrentStartTime = -1;
            }
            SmartDashboard.putString("Resistance Status", "Normal Current");
        }
    }
    
    public void resetResistanceDetection() {
        shooterResistanceDetected = false;
        shooterHighCurrentStartTime = -1;
        System.out.println("Resistance detection manually reset - Motors can now run");
    }
    
    public double getShooterCurrent() {
        return shooterMotor.getSupplyCurrent().getValueAsDouble();
    }
    
    public boolean isShooterResistanceDetected() {
        return shooterResistanceDetected;
    }
    
}