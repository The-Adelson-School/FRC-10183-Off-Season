package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class IncreaseCommand extends Command {
    private final ElevatorSubsystem elevator;
    
    /**
     * Creates a new IncreaseCommand.
     * @param elevatorSubsystem The elevator subsystem used by this command
     */
    public IncreaseCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevator = elevatorSubsystem;
        addRequirements(elevator);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.increaseStage();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // This command executes instantly, so nothing needed here
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true; // This command finishes immediately after increasing stage
    }
}
