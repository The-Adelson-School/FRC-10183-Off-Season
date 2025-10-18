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
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {

    TalonFX elevMotorLeader;
    TalonFX elevMotorFollower;
    TalonFX intakeMotor;
    TalonFX shooterMotor;
    TalonFX algaeKickerMotor;
    TalonFX Climber;
    // COMMENTED OUT: Cage motors
    /*
    TalonFX cageMotor; // NEW: Cage motor
    TalonFX cageClimbMotor; // NEW: Cage climb motor
    */
    
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    // COMMENTED OUT: Cage motion magic request
    /*
    private final MotionMagicVoltage cageMotionMagicRequest = new MotionMagicVoltage(0); // NEW: For cage motor
    */
    
    // Resistance detection - SHOOTER ONLY
    private double shooterHighCurrentStartTime = -1;
    private boolean shooterResistanceDetected = false;
    
    // Average current tracking
    private double avgCurrent = 0.0;
    
    // Manual shooter override
    private boolean manualShooterOverride = false;
    
    // Algae mode tracking to disable shooter during algae operations
    private boolean algaeKickerActive = false;

    // Manual control flags for autonomous
    private boolean manualShooterControl = false;
    private boolean manualIntakeControl = false;
    private boolean manualShooterState = false;
    private boolean manualIntakeState = false;
    
    // COMMENTED OUT: Cage Motor State Tracking
    /*
    private boolean cageMotorSpinning = false;
    private double cageHighCurrentStartTime = -1;
    private boolean cageResistanceDetected = false;
    private boolean cageRotationComplete = false;
    private double cageStartPosition = 0.0;
    */
    
    int stage = 0;
    
    public ElevatorSubsystem(int elevLeaderID, int elevFollowerID, int intakeID, int shooterID, int algaeKickerID){
        elevMotorLeader = new TalonFX(elevLeaderID, "CANivore");
        elevMotorFollower = new TalonFX(elevFollowerID, "CANivore");
        intakeMotor = new TalonFX(intakeID, "CANivore");
        shooterMotor = new TalonFX(shooterID, "CANivore");
        algaeKickerMotor = new TalonFX(algaeKickerID, "CANivore");
        // COMMENTED OUT: Cage motor initialization
        /*
        cageMotor = new TalonFX(ElevatorConstants.CAGE_MOTOR_ID, "CANivore"); // NEW
        cageClimbMotor = new TalonFX(ElevatorConstants.CAGE_CLIMB_ID, "CANivore"); // NEW
        */
        
        configureMotionMagic();
        
        elevMotorLeader.setNeutralMode(NeutralModeValue.Brake);
        elevMotorFollower.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
        algaeKickerMotor.setNeutralMode(NeutralModeValue.Brake);
        // COMMENTED OUT: Cage motor brake mode
        /*
        cageMotor.setNeutralMode(NeutralModeValue.Brake); // NEW
        cageClimbMotor.setNeutralMode(NeutralModeValue.Brake); // NEW
        */
        
        // Follower motor follows leader
        elevMotorFollower.setControl(new Follower(elevLeaderID, false));
        
        elevMotorLeader.setPosition(0);
        elevMotorFollower.setPosition(0);
        intakeMotor.setPosition(0);
        shooterMotor.setPosition(0);
        algaeKickerMotor.setPosition(0);
        // COMMENTED OUT: Cage motor position reset
        /*
        cageMotor.setPosition(0); // NEW
        cageClimbMotor.setPosition(0); // NEW
        */
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
        intakeCurrentConfig.withSupplyCurrentLimit(30.0);
        intakeCurrentConfig.withStatorCurrentLimit(60.0);
        
        intakeMotor.getConfigurator().apply(intakeConfig);
        
        // Configuration for shooter motor (30A supply, 35A stator)
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs shooterCurrentConfig = shooterConfig.CurrentLimits;
        shooterCurrentConfig.withSupplyCurrentLimitEnable(true);
        shooterCurrentConfig.withStatorCurrentLimitEnable(true);
        shooterCurrentConfig.withSupplyCurrentLimit(30.0);
        shooterCurrentConfig.withStatorCurrentLimit(35.0);
        
        shooterMotor.getConfigurator().apply(shooterConfig);
        
        // Configuration for algae kicker motor (30A supply, 60A stator)
        TalonFXConfiguration algaeKickerConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs algaeKickerCurrentConfig = algaeKickerConfig.CurrentLimits;
        algaeKickerCurrentConfig.withSupplyCurrentLimitEnable(true);
        algaeKickerCurrentConfig.withStatorCurrentLimitEnable(true);
        algaeKickerCurrentConfig.withSupplyCurrentLimit(60.0);
        algaeKickerCurrentConfig.withStatorCurrentLimit(80.0);
        
        algaeKickerMotor.getConfigurator().apply(algaeKickerConfig);

        // COMMENTED OUT: Configuration for cage motors
        /*
        // NEW: Configuration for cage motor with Motion Magic
        TalonFXConfiguration cageConfig = new TalonFXConfiguration();
        
        // Motion Magic configuration for cage motor
        MotionMagicConfigs cageMotionMagicConfigs = cageConfig.MotionMagic;
        cageMotionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CAGE_MOTION_MAGIC_CRUISE_VELOCITY;
        cageMotionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.CAGE_MOTION_MAGIC_ACCELERATION;
        
        // PID configuration for cage motor
        Slot0Configs cageSlot0Configs = cageConfig.Slot0;
        cageSlot0Configs.kP = ElevatorConstants.CAGE_MOTION_MAGIC_KP;
        cageSlot0Configs.kI = ElevatorConstants.CAGE_MOTION_MAGIC_KI;
        cageSlot0Configs.kD = ElevatorConstants.CAGE_MOTION_MAGIC_KD;
        cageSlot0Configs.kV = ElevatorConstants.CAGE_MOTION_MAGIC_KV;
        cageSlot0Configs.kS = ElevatorConstants.CAGE_MOTION_MAGIC_KS;
        cageSlot0Configs.kA = ElevatorConstants.CAGE_MOTION_MAGIC_KA;
        cageSlot0Configs.kG = ElevatorConstants.CAGE_MOTION_MAGIC_KG;
        
        // Current limits for cage motor
        CurrentLimitsConfigs cageCurrentConfig = cageConfig.CurrentLimits;
        cageCurrentConfig.withSupplyCurrentLimitEnable(true);
        cageCurrentConfig.withStatorCurrentLimitEnable(true);
        cageCurrentConfig.withSupplyCurrentLimit(ElevatorConstants.CAGE_SUPPLY_CURRENT_LIMIT);
        cageCurrentConfig.withStatorCurrentLimit(ElevatorConstants.CAGE_STATOR_CURRENT_LIMIT);
        
        cageMotor.getConfigurator().apply(cageConfig);
        
        // Configuration for cage climb motor (same current limits, no Motion Magic needed)
        TalonFXConfiguration cageClimbConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs cageClimbCurrentConfig = cageClimbConfig.CurrentLimits;
        cageClimbCurrentConfig.withSupplyCurrentLimitEnable(true);
        cageClimbCurrentConfig.withStatorCurrentLimitEnable(true);
        cageClimbCurrentConfig.withSupplyCurrentLimit(ElevatorConstants.CAGE_SUPPLY_CURRENT_LIMIT);
        cageClimbCurrentConfig.withStatorCurrentLimit(ElevatorConstants.CAGE_STATOR_CURRENT_LIMIT);
        
        cageClimbMotor.getConfigurator().apply(cageClimbConfig);
        */
    }

    public int getStage(){
        return stage;
    }

    private void goToHeight(int elevSetpoint) {
        double targetRotations = (double)elevSetpoint / ElevatorConstants.COUNTS_PER_ROTATION;
        elevMotorLeader.setControl(motionMagicRequest.withPosition(targetRotations));
    }
    
    public boolean isElevatorAtTarget(int targetCounts, double tolerance) {
        double currentCounts = elevMotorLeader.getPosition().getValueAsDouble() * ElevatorConstants.COUNTS_PER_ROTATION;
        return Math.abs(currentCounts - targetCounts) <= tolerance;
    }
    
    public boolean isElevatorAtTarget(int targetCounts) {
        return isElevatorAtTarget(targetCounts, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE);
    }

    public void setIntakeSpeed(double speed) {
        if (stage != 0) {
            intakeMotor.set(speed);
        }
    }
    
    public void setShooterSpeed(double speed) {
        if (speed != 0.0) {
            shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        } else {
            shooterMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        
        if (stage == 0 && shooterResistanceDetected) {
            shooterMotor.set(ElevatorConstants.SHOOTER_STOP);
        } else {
            shooterMotor.set(speed);
        }
    }
    
    public void setManualShooterOverride(boolean override) {
        this.manualShooterOverride = override;
    }

    public void increaseStage(){
        if(stage < 3){
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
        stage = Math.max(0, Math.min(3, targetStage));
        
        if (previousStage == 0 && stage != 0) {
            resetResistanceDetection();
        }
        
        if (stage == 0) {
            goToHeight(ElevatorConstants.STOWED_LEVEL);
        } else if (stage == 1) {
            goToHeight(ElevatorConstants.LEVEL_ONE);
            setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
        } else if (stage == 2) {
            goToHeight(ElevatorConstants.LEVEL_TWO);
            setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
        } else if (stage == 3) {
            goToHeight(ElevatorConstants.LEVEL_THREE);
            setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
        }
    }
    
    public void goToAlgaePositionA() {
        goToHeight(ElevatorConstants.ALGAE_POSITION_A);
    }
    
    public void goToAlgaePositionB() {
        goToHeight(ElevatorConstants.ALGAE_POSITION_B);
    }
    
    public boolean isElevatorAtAlgaePositionA() {
        return isElevatorAtTarget(ElevatorConstants.ALGAE_POSITION_A);
    }
    
    public boolean isElevatorAtAlgaePositionB() {
        return isElevatorAtTarget(ElevatorConstants.ALGAE_POSITION_B);
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

    public void setAlgaeKickerSpeed(double speed) {
        algaeKickerMotor.set(speed);
        
        boolean wasActive = algaeKickerActive;
        algaeKickerActive = (speed != 0.0);
        
        if (algaeKickerActive && !wasActive) {
            setShooterBrakeMode(true);
            shooterMotor.set(0.0);
            System.out.println("ALGAE KICKER ACTIVATED: Shooter FORCED to BRAKE MODE");
        } else if (!algaeKickerActive && wasActive) {
            System.out.println("ALGAE KICKER DEACTIVATED: Shooter can resume normal operations");
        }
    }
    
    public void startAlgaeKicker() {
        setAlgaeKickerSpeed(ElevatorConstants.ALGAE_KICKER_ON);
        System.out.println("ALGAE KICKER STARTED");
    }
    
    public void stopAlgaeKicker() {
        setAlgaeKickerSpeed(ElevatorConstants.ALGAE_KICKER_STOP);
        System.out.println("ALGAE KICKER STOPPED");
    }
    
    public boolean isAlgaeKickerActive() {
        return algaeKickerActive;
    }
    
    public void setShooterBrakeMode(boolean brake) {
        if (brake) {
            shooterMotor.setNeutralMode(NeutralModeValue.Brake);
            System.out.println("SHOOTER: Set to BRAKE mode");
        } else {
            shooterMotor.setNeutralMode(NeutralModeValue.Coast);
            System.out.println("SHOOTER: Set to COAST mode");
        }
    }
    
    public void lockShooterPosition() {
        setShooterBrakeMode(true);
        shooterMotor.set(0.0);
        System.out.println("SHOOTER LOCKED");
    }
    
    public void releaseShooterLock() {
        setShooterBrakeMode(false);
        System.out.println("SHOOTER RELEASED");
    }

    public void enableManualShooterIntakeControl() {
        manualShooterControl = true;
        manualIntakeControl = true;
        manualShooterState = false;
        manualIntakeState = false;
        System.out.println("AUTONOMOUS: Manual shooter/intake control ENABLED");
    }
    
    public void disableManualShooterIntakeControl() {
        manualShooterControl = false;
        manualIntakeControl = false;
        manualShooterState = false;
        manualIntakeState = false;
        System.out.println("AUTONOMOUS: Manual shooter/intake control DISABLED");
    }
    
    public void startShooterAndIntakeManual() {
        if (manualShooterControl && manualIntakeControl) {
            if (isAtStowedLevel()) {
                manualShooterState = true;
                manualIntakeState = true;
                System.out.println("AUTONOMOUS: Started shooter and intake (manual control)");
            } else {
                System.out.println("AUTONOMOUS WARNING: Cannot start shooter/intake - robot not at STOWED LEVEL");
            }
        } else {
            System.out.println("AUTONOMOUS ERROR: Manual control not enabled");
        }
    }
    
    public void stopShooterAndIntakeManual() {
        if (manualShooterControl && manualIntakeControl) {
            manualShooterState = false;
            manualIntakeState = false;
            System.out.println("AUTONOMOUS: Stopped shooter and intake (manual control)");
        } else {
            System.out.println("AUTONOMOUS ERROR: Manual control not enabled");
        }
    }
    
    public boolean isAtStowedLevel() {
        return stage == 0 && isElevatorAtTarget(ElevatorConstants.STOWED_LEVEL);
    }
    
    public boolean isManualControlEnabled() {
        return manualShooterControl && manualIntakeControl;
    }

    public void climberteleop(double speed){
        Climber.set(speed);
    }

    // COMMENTED OUT: Cage Motor Control Methods
    /*
    public void startCageMotorSpin() {
        if (!cageMotorSpinning && !cageResistanceDetected && cageRotationComplete) {
            cageMotorSpinning = true;
            cageResistanceDetected = false;
            cageHighCurrentStartTime = -1;
            cageRotationComplete = false;
            cageMotor.set(ElevatorConstants.CAGE_SPIN_SPEED);
            System.out.println("CAGE MOTOR: Started spinning at full speed");
        }
    }
    
    public void stopCageMotor() {
        cageMotorSpinning = false;
        cageResistanceDetected = false;
        cageHighCurrentStartTime = -1;
        cageMotor.set(ElevatorConstants.CAGE_STOP);
        System.out.println("CAGE MOTOR: Stopped");
    }
    
    public void setCageClimbSpeed(double speed) {
        cageClimbMotor.set(speed);
    }
    
    public double getCageMotorCurrent() {
        return cageMotor.getSupplyCurrent().getValueAsDouble();
    }
    
    public boolean isCageMotorSpinning() {
        return cageMotorSpinning;
    }
    
    public boolean isCageResistanceDetected() {
        return cageResistanceDetected;
    }
    
    public boolean isCageRotationComplete() {
        return cageRotationComplete;
    }
    */

    @Override
    public void periodic() {
        boolean shooterShouldRun = false;
        
        if (algaeKickerActive) {
            shooterShouldRun = false;
            shooterMotor.setNeutralMode(NeutralModeValue.Brake);
            shooterMotor.set(0.0);
            shooterResistanceDetected = false;
            return;
        } else {
            if (manualShooterControl) {
                shooterShouldRun = manualShooterState;
            } else {
                if (stage == 0) {
                    shooterShouldRun = true;
                } else {
                    if (manualShooterOverride) {
                        shooterShouldRun = true;
                    }
                }
            }
        }
        
        if (shooterShouldRun && !algaeKickerActive) {
            updateCurrentDetection();
        } else {
            shooterResistanceDetected = false;
        }
        
        if (shooterShouldRun && !shooterResistanceDetected && !algaeKickerActive) {
            shooterMotor.setNeutralMode(NeutralModeValue.Coast);
            shooterMotor.set(ElevatorConstants.SHOOTER_ON);
            
            if (stage == 0) {
                intakeMotor.set(ElevatorConstants.INTAKE_OUT);
            } else {
                if (manualIntakeControl) {
                    if (manualIntakeState) {
                        intakeMotor.set(ElevatorConstants.INTAKE_OUT);
                    } else {
                        intakeMotor.set(ElevatorConstants.INTAKE_STOP);
                    }
                } else {
                    intakeMotor.set(ElevatorConstants.INTAKE_STOP);
                }
            }
            
        } else {
            if (!algaeKickerActive) {
                shooterMotor.setNeutralMode(NeutralModeValue.Brake);
            }
            
            shooterMotor.set(ElevatorConstants.SHOOTER_STOP);
            
            if (stage == 0) {
                intakeMotor.set(ElevatorConstants.INTAKE_OUT);
            } else {
                if (manualIntakeControl) {
                    if (manualIntakeState) {
                        intakeMotor.set(ElevatorConstants.INTAKE_OUT);
                    } else {
                        intakeMotor.set(ElevatorConstants.INTAKE_STOP);
                    }
                } else {
                    intakeMotor.set(ElevatorConstants.INTAKE_STOP);
                }
            }
        }

        // COMMENTED OUT: Cage Motor Logic
        /*
        if (cageMotorSpinning && !cageResistanceDetected) {
            updateCageCurrentDetection();
        }
        
        // Handle cage motor state transitions
        if (cageMotorSpinning && cageResistanceDetected && !cageRotationComplete) {
            // Stop spinning and start controlled rotation
            cageMotorSpinning = false;
            cageStartPosition = cageMotor.getPosition().getValueAsDouble();
            
            // Calculate target position: current + (degrees / 360) / gear_ratio
            double targetRotations = cageStartPosition + (ElevatorConstants.CAGE_ROTATION_DEGREES / 360.0) / ElevatorConstants.CAGE_GEAR_RATIO;
            cageMotor.setControl(cageMotionMagicRequest.withPosition(targetRotations));
            
            System.out.println("CAGE MOTOR: Resistance detected, starting " + ElevatorConstants.CAGE_ROTATION_DEGREES + " degree rotation");
            System.out.println("CAGE MOTOR: From position " + cageStartPosition + " to " + targetRotations);
        }
        
        // Check if rotation is complete
        if (cageResistanceDetected && !cageRotationComplete) {
            double currentPosition = cageMotor.getPosition().getValueAsDouble();
            double targetPosition = cageStartPosition + (ElevatorConstants.CAGE_ROTATION_DEGREES / 360.0) / ElevatorConstants.CAGE_GEAR_RATIO;
            
            if (Math.abs(currentPosition - targetPosition) < 0.02) { // Within 0.02 rotations tolerance
                cageRotationComplete = true;
                System.out.println("CAGE MOTOR: " + ElevatorConstants.CAGE_ROTATION_DEGREES + " degree rotation complete");
            }
        }
        */
    }
    
    private void updateCurrentDetection() {
        double currentTime = Timer.getFPGATimestamp();
        double shooterCurrent = getShooterCurrent();
        avgCurrent = shooterCurrent;
        
        if (ElevatorConstants.SHOOTER_CURRENT_THRESHOLD <= 0) {
            shooterResistanceDetected = false;
            shooterHighCurrentStartTime = -1;
            return;
        }
        
        if (shooterResistanceDetected) {
            return;
        }
        
        if (shooterCurrent > ElevatorConstants.SHOOTER_CURRENT_THRESHOLD) {
            if (shooterHighCurrentStartTime < 0) {
                shooterHighCurrentStartTime = currentTime;
                System.out.println("HIGH CURRENT TIMER STARTED: " + shooterCurrent + "A");
            } else {
                double highCurrentDuration = currentTime - shooterHighCurrentStartTime;
                
                if (highCurrentDuration >= ElevatorConstants.CURRENT_DETECTION_TIME) {
                    shooterResistanceDetected = true;
                    System.out.println("RESISTANCE DETECTED - Motors LOCKED OFF");
                }
            }
        } else {
            if (shooterHighCurrentStartTime >= 0) {
                System.out.println("CURRENT DROPPED TO NORMAL: " + shooterCurrent + "A");
                shooterHighCurrentStartTime = -1;
            }
        }
    }

    // COMMENTED OUT: Cage Motor Current Detection
    /*
    private void updateCageCurrentDetection() {
        double currentTime = Timer.getFPGATimestamp();
        double cageCurrent = getCageMotorCurrent();
        
        if (ElevatorConstants.CAGE_CURRENT_THRESHOLD <= 0) {
            return; // Disabled
        }
        
        if (cageCurrent > ElevatorConstants.CAGE_CURRENT_THRESHOLD) {
            if (cageHighCurrentStartTime < 0) {
                cageHighCurrentStartTime = currentTime;
                System.out.println("CAGE MOTOR: High current detected: " + cageCurrent + "A - Starting timer");
            } else {
                double highCurrentDuration = currentTime - cageHighCurrentStartTime;
                
                if (highCurrentDuration >= ElevatorConstants.CAGE_RESISTANCE_TIME) {
                    cageResistanceDetected = true;
                    System.out.println("CAGE MOTOR: Resistance confirmed after " + ElevatorConstants.CAGE_RESISTANCE_TIME + " seconds");
                }
            }
        } else {
            if (cageHighCurrentStartTime >= 0) {
                System.out.println("CAGE MOTOR: Current dropped to normal: " + cageCurrent + "A - Resetting timer");
                cageHighCurrentStartTime = -1;
            }
        }
    }
    */
    
    public void resetResistanceDetection() {
        shooterResistanceDetected = false;
        shooterHighCurrentStartTime = -1;
        System.out.println("Resistance detection manually reset");
    }
    
    public double getShooterCurrent() {
        return shooterMotor.getSupplyCurrent().getValueAsDouble();
    }
    
    public boolean isShooterResistanceDetected() {
        return shooterResistanceDetected;
    }
    
    private int getCurrentStageTarget() {
        switch (stage) {
            case 0: return ElevatorConstants.STOWED_LEVEL;
            case 1: return ElevatorConstants.LEVEL_ONE;
            case 2: return ElevatorConstants.LEVEL_TWO;
            case 3: return ElevatorConstants.LEVEL_THREE;
            default: return ElevatorConstants.STOWED_LEVEL;
        }
    }
    
    public void emergencyStop() {
        elevMotorLeader.set(0);
        intakeMotor.set(0);
        shooterMotor.setNeutralMode(NeutralModeValue.Brake);
        shooterMotor.set(0);
        algaeKickerMotor.set(0);
        // COMMENTED OUT: Cage motor emergency stop
        /*
        cageMotor.set(0); // NEW
        cageClimbMotor.set(0); // NEW
        */
        algaeKickerActive = false;
        resetResistanceDetection();
        System.out.println("EMERGENCY STOP: All motors stopped");
    }
    
    public Command moveToStageCommand(int targetStage) {
        return Commands.runOnce(() -> engageStage(targetStage))
            .andThen(Commands.waitUntil(() -> isElevatorAtTarget(getCurrentStageTarget())))
            .withName("MoveToStage" + targetStage);
    }
    
}

