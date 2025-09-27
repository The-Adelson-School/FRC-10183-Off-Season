package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    //Elevator Math
    public static final int COUNTS_PER_ROTATION = 3402 * 4; // 3402 base counts * 4:1 gear ratio = 13608
    public static final int MAX_EXTENTION_INCHES = 11;
    public static final int INCHES_PER_ROTATION = 4; //replace with actual when determined
    public static final int COUNTS_PER_INCH = COUNTS_PER_ROTATION/INCHES_PER_ROTATION;
    
    //Elevator Set Points (Only 3 stages: 0, 1, 2)
    public static final int STOWED_LEVEL = 0;      // Stage 0
    public static final int LEVEL_ONE = 0;         // Stage 1 - change when actual determined
    public static final int LEVEL_TWO = 0;         // Stage 2 - change when actual determined
    
    int stage = 0;
    TalonFX elevMotor = new TalonFX(6);
    TalonFX intakeMotor = new TalonFX(7);
    TalonFX intake2Motor = new TalonFX(8);
    TalonFX intake3Motor = new TalonFX(9);
    TalonFX intake4Motor = new TalonFX(10);

    public Elevator() {
        // Configure all motors for brake mode
        elevMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intake2Motor.setNeutralMode(NeutralModeValue.Brake);
        intake3Motor.setNeutralMode(NeutralModeValue.Brake);
        intake4Motor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void goToHeight(int elevSetpoint){
        elevMotor.setPosition(elevSetpoint);
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
        if(stage < 2){  // Changed from 4 to 2
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
            goToHeight(LEVEL_ONE);
        }else if(stage == 2){
            goToHeight(LEVEL_TWO);
        }
    }
}
