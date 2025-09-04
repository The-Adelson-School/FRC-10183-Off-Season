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

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed); // Start spinning
    
        // Pause for .45 seconds
        try {
            Thread.sleep(450);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    
        intakeMotor.set(0);
    }

    public void setIntake2Speed(double speed) {
        intake2Motor.set(speed); // Start spinning
    
        // Pause for .45 seconds
        try {
            Thread.sleep(450);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    
        intake2Motor.set(0);
    }
    
    public void setIntake3Speed(double speed) {
        intake3Motor.set(speed); // Start spinning
    
        // Pause for .45 seconds
        try {
            Thread.sleep(450);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    
        intake3Motor.set(0);
    }

    public void setIntake4Speed(double speed) {
        intake4Motor.set(speed); // Start spinning
    
        // Pause for .45 seconds
        try {
            Thread.sleep(450);
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
