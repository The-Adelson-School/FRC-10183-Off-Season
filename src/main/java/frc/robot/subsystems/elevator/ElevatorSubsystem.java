package frc.robot.subsystems.elevator; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command; 
import static edu.wpi.first.units.Units.Seconds;

public class ElevatorSubsystem extends SubsystemBase {

    TalonFX elevMotor;
    TalonFX intakeMotor;
    
    public ElevatorSubsystem(int elevID, int intakeID, int unused1, int unused2, int unused3){
        elevMotor = new TalonFX(elevID);
        intakeMotor = new TalonFX(intakeID);
        
        // Configure motors for brake mode
        elevMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Reset encoder positions to zero
        elevMotor.setPosition(0);
        intakeMotor.setPosition(0);
    }
    
    int stage = 0;
    
    PIDController elevController = new PIDController(0.00003, 0.000000, 0.00000); //currently needs testing

    public int getStage(){
        return stage;
    }

    private void goToHeight(int elevSetpoint){
        // Get current position from TalonFX encoder (in rotations, multiply by gear ratio for actual counts)
        double currentPosition = elevMotor.getPosition().getValueAsDouble() * ElevatorConstants.COUNTS_PER_ROTATION;
        elevMotor.set(MathUtil.clamp(elevController.calculate(currentPosition, elevSetpoint), -1.0, 1.0));
    }

    private PIDController intakePIDController;
    private double targetPosition = 0; 
    private double targetVelocity = 0; 
    private boolean usePositionControl = true;

    // In constructor, initialize PID controller
    public ElevatorSubsystem(){
        // Initialize PID controller with tuned constants.
        intakePIDController = new PIDController(0.1, 0.0, 0.0);
        intakePIDController.setTolerance(0.01); // Set PID tolerances
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    // Helper methods to get encoder values
    private double getCurrentPosition() {
        return intakeMotor.getPosition().getValueAsDouble();
    }

    private double getCurrentVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    // Method to check if intake is at target
    public boolean isAtTarget() {
        return intakePIDController.atSetpoint();
    }

    // Method to stop intake
    public void stopIntake() {
        intakeMotor.set(0);
        intakePIDController.reset();
    }

    public void increaseStage(){
        if(stage < 2){  // Changed from 5 to 2 (stages 0, 1, 2)
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
        
        if (stage == 0) {
            goToHeight(ElevatorConstants.STOWED_LEVEL);
            // Auto-start intake when in stage 0
            intakeMotor.set(ElevatorConstants.INTAKE_IN);
        } else if (stage == 1) {
            goToHeight(ElevatorConstants.LEVEL_ONE);
            // Auto-stop intake when not in stage 0
            intakeMotor.set(ElevatorConstants.INTAKE_STOP);
        } else if (stage == 2) {
            goToHeight(ElevatorConstants.LEVEL_TWO);
            // Auto-stop intake when not in stage 0
            intakeMotor.set(ElevatorConstants.INTAKE_STOP);
        }
    }
    
    public void defaultCommand() {
        engageStage(); 
        // Remove the manual intake stop since it's now handled in engageStage()
    }

    public void autonomousCommand(){
        stage = 1;  // Changed from 2 to 1
        engageStage();
        
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        setIntakeSpeed(ElevatorConstants.INTAKE_OUT);
        
        stage = 0;
        engageStage();
        
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        // Add telemetry for debugging
        SmartDashboard.putNumber("Elevator Position", elevMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Position", intakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current", elevMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Current", intakeMotor.getStatorCurrent().getValueAsDouble());
    }
}
