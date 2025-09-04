package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class DecreaseCommand extends Command {
    ElevatorSubsystem elevator = new ElevatorSubsystem(6, 8, 8, 8, 8);
    int curStage = elevator.getStage();

    @Override
    public void initialize(){
        
        elevator.decreaseStage();
        elevator.engageStage();
    }

    @Override 
    public boolean isFinished(){
        if(elevator.getStage() < curStage || curStage == 0){
            return true;
        }
        return false;
    }
}
