package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class IncreaseCommand extends Command {
    ElevatorSubsystem elevator = new ElevatorSubsystem(6, 8, 8, 8, 8);
    int curStage = elevator.getStage();

    @Override
    public void initialize(){
        
        elevator.increaseStage();
        elevator.engageStage();
    }

    @Override 
    public boolean isFinished(){
        if(elevator.getStage() > curStage || curStage == 4){
            return true;
        }
        return false;
    }
}
