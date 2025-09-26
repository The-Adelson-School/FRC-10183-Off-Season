package frc.robot.subsystems.elevator; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.utility.LazyCANSparkMax;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command; 
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel;



public class ElevatorSubsystem extends SubsystemBase {

    LazyCANSparkMax elevMotor;
    LazyCANSparkMax intakeMotor;
    LazyCANSparkMax intake2Motor;
    LazyCANSparkMax intake3Motor;
    LazyCANSparkMax intake4Motor;
    
    public ElevatorSubsystem(int elevID, int intakeID, int intake2ID, int intake3ID, int intake4ID){
        elevMotor = new LazyCANSparkMax(elevID, SparkLowLevel.MotorType.kBrushless);
        intakeMotor = new LazyCANSparkMax(intakeID, SparkLowLevel.MotorType.kBrushless);
        intake2Motor = new LazyCANSparkMax(intake2ID, SparkLowLevel.MotorType.kBrushless);
        intake3Motor = new LazyCANSparkMax(intake3ID, SparkLowLevel.MotorType.kBrushless);
        intake4Motor = new LazyCANSparkMax(intake4ID, SparkLowLevel.MotorType.kBrushless);
    }
    
    
    int stage = 0;
    
    PIDController elevController = new PIDController(0.00003, 0.000000, 0.00000); //currently needs testing

    public int getStage(){
        return stage;
    }

    private void goToHeight(int elevSetpoint){
        elevMotor.set(MathUtil.clamp(elevController.calculate(elevMotor.getEncoder().getPosition()*ElevatorConstants.COUNTS_PER_ROTATION, elevSetpoint), -1.0, 1.0));
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
        if (usePositionControl){
            // Position-based control
            targetPosition = getCurrentPosition() + (speed * ElevatorConstants.INTAKE_ROTATION_DISTANCE);
            intakePIDController.setSetpoint(targetPosition); 
        }else{
            // Velocity-based control
            targetVelocity = speed; 
            intakePIDController.setSetpoint(targetVelocity);
        }
    }

    // Call this method periodically (in periodic() method)
    public void updateIntakePID(){
        double pidOutput; // Position control
        if (usePositionControl){
            pidOutput = intakePIDController.calculate(getCurrentPosition()); 
            // Stop motor when at target position
            if (intakePIDController.atSetpoint()){
                intakeMotor.set(0); 
                return;
            }
        }else{
            // Velocity control
            pidOutput = intakePIDController.calculate(getCurrentVelocity()); 
        }

        // Apply output limits
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        intakeMotor.set(pidOutput);
    }

    // Helper methods to get encoder values
    private double getCurrentPosition() {
        // Replace with your actual encoder method
        return intakeMotor.getEncoder().getPosition();
    }

    private double getCurrentVelocity() {
        // Replace with your actual encoder method
        return intakeMotor.getEncoder().getVelocity();
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

    public void setIntake2Speed(double speed) {
        intake2Motor.set(speed); // Start spinning
    
        // Pause for .45 seconds
        try {
            Thread.sleep((long)ElevatorConstants.INTAKE2_SPEED_MS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    
        intake2Motor.set(0);
    }
    
    public void setIntake3Speed(double speed) {
        intake3Motor.set(speed); // Start spinning
    
        // Pause for .45 seconds
        try {
            Thread.sleep((long)ElevatorConstants.INTAKE3_SPEED_MS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    
        intake3Motor.set(0);
    }

    public void setIntake4Speed(double speed) {
        intake4Motor.set(speed); // Start spinning
    
        // Pause for .45 seconds
        try {
            Thread.sleep((long)ElevatorConstants.INTAKE4_SPEED_MS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    
        intake4Motor.set(0);
    }
    
    


    public void increaseStage(){
        if(stage < 5){
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
        // Use the current stage variable as the target stage
        engageStage(stage);
    }
    
    // Existing engageStage method with an argument
    public void engageStage(int targetStage) {
        stage = targetStage;
        SmartDashboard.putNumber("Stage", stage);
        if (stage == 0) {
            goToHeight(ElevatorConstants.STOWED_LEVEL);
        } else if (stage == 1) {
            goToHeight(ElevatorConstants.CORAL_STATION);
        } else if (stage == 2) {
            goToHeight(ElevatorConstants.LEVEL_ONE);
        } else if (stage == 3) {
            goToHeight(ElevatorConstants.LEVEL_TWO);
        } else if (stage == 4) {
            goToHeight(ElevatorConstants.LEVEL_THREE);
        } else if (stage == 5) {
            goToHeight(ElevatorConstants.LEVEL_FOUR);
        }
    }
    
    public void defaultCommand() {
        engageStage(); 
        // Remove this line to stop overriding intake motor speed:
        intakeMotor.set(ElevatorConstants.INTAKE_STOP); 
        intake2Motor.set(ElevatorConstants.INTAKE2_STOP);
        intake3Motor.set(ElevatorConstants.INTAKE3_STOP);
        intake4Motor.set(ElevatorConstants.INTAKE4_STOP);

        
    }

    public void autonomousCommand(){
        stage = 2;
        engageStage();
               // Pause for .45 seconds
               try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        setIntakeSpeed(ElevatorConstants.INTAKE_OUT);
        setIntake2Speed(ElevatorConstants.INTAKE2_OUT);
        setIntake3Speed(ElevatorConstants.INTAKE3_OUT);
        setIntake4Speed(ElevatorConstants.INTAKE4_OUT);
        stage = 0;
        engageStage();
               // Pause for .45 seconds
               try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
    }
    

}
