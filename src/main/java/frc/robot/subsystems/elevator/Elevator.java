package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.LazyCANSparkMax;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkLowLevel;



public class Elevator extends SubsystemBase {
    //Elevator Math
    public static final int COUNTS_PER_ROTATION = 3402;
    public static final int MAX_EXTENTION_INCHES = 11;
    public static final int INCHES_PER_ROTATION = 4; //replace with actual when determined
    public static final int COUNTS_PER_INCH = COUNTS_PER_ROTATION/INCHES_PER_ROTATION;
    
    
    //Elevator Set Points
    public static final int STOWED_LEVEL = 0;
    public static final int CORAL_STATION = 0; // change when actual determined
    public static final int LEVEL_ONE = 0; // change when actual determined
    public static final int LEVEL_TWO = 0; // change when actual determined
    public static final int LEVEL_THREE = 0; // change when actual determined
    
    int stage = 0;
    LazyCANSparkMax elevMotor = new LazyCANSparkMax(6, SparkLowLevel.MotorType.kBrushless);
    LazyCANSparkMax intakeMotor = new LazyCANSparkMax(7, SparkLowLevel.MotorType.kBrushless);
    LazyCANSparkMax intake2Motor = new LazyCANSparkMax(8, SparkLowLevel.MotorType.kBrushless);
    LazyCANSparkMax intake3Motor = new LazyCANSparkMax(9, SparkLowLevel.MotorType.kBrushless);
    LazyCANSparkMax intake4Motor = new LazyCANSparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private void goToHeight(int elevSetpoint){
        elevMotor.getEncoder().setPosition(elevSetpoint);
    }

    private void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    private void setIntake2Speed(double speed){
        intake2Motor.set(speed);
    }

    private void setIntake3Speed(double speed){
        intake3Motor.set(speed);
    }

    private void setIntake4Speed(double speed){
        intake4Motor.set(speed);
    }

    private void increaseStage(){
        if(stage < 4){
            stage++;
        }
    }

    private void decreaseStage(){
        if(stage > 0){
            stage--;
        }
    }

    @Override
    public void periodic(){
        if(stage == 0){
            goToHeight(STOWED_LEVEL);
        }else if(stage == 1){
            goToHeight(CORAL_STATION);
        }
        else if(stage == 2){
            goToHeight(LEVEL_ONE);
        }else if(stage == 3){
            goToHeight(LEVEL_TWO);
        }else if(stage == 4){
            goToHeight(LEVEL_THREE);
        }
  }

    
}
