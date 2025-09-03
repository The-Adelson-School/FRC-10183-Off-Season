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
    
    public ElevatorSubsystem(int elevID, int wristID, int intakeID){
        elevMotor = new LazyCANSparkMax(elevID, SparkLowLevel.MotorType.kBrushless);
        intakeMotor = new LazyCANSparkMax(intakeID, SparkLowLevel.MotorType.kBrushless);
    }
    
    
    int stage = 0;
    
    PIDController elevController = new PIDController(0.00003, 0.000000, 0.00000); //currently needs testing

    public int getStage(){
        return stage;
    }

    private void goToHeight(int elevSetpoint, int wristSetpoint){
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
            goToHeight(ElevatorConstants.STOWED_LEVEL, ElevatorConstants.STOWED_ANGLE);
        } else if (stage == 1) {
            goToHeight(ElevatorConstants.CORAL_STATION, ElevatorConstants.CORAL_ANGLE);
        } else if (stage == 2) {
            goToHeight(ElevatorConstants.LEVEL_ONE, ElevatorConstants.SCORING_ANGLE);
        } else if (stage == 3) {
            goToHeight(ElevatorConstants.LEVEL_TWO, ElevatorConstants.SCORING_ANGLE);
        } else if (stage == 4) {
            goToHeight(ElevatorConstants.LEVEL_THREE, ElevatorConstants.SCORING_ANGLE);
        } else if (stage == 5) {
            goToHeight(ElevatorConstants.LEVEL_FOUR, ElevatorConstants.LEVEL_FOUR_ANGLE);
        }
    }
    
    public void defaultCommand() {
        engageStage(); 
        // Remove this line to stop overriding intake motor speed:
        intakeMotor.set(ElevatorConstants.INTAKE_STOP); 
        
        

        
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
