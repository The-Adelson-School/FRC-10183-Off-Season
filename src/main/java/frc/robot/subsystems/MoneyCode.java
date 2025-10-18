package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoneyCode {
    
    private final SwerveSubsystem drivebase;
    private final ElevatorSubsystem elevator;
    
    public MoneyCode(SwerveSubsystem drivebase, ElevatorSubsystem elevator) {
        this.drivebase = drivebase;
        this.elevator = elevator;
    }

    public Command Forward() {
        return Commands.sequence(
            Commands.run(() -> drivebase.drive(new Translation2d(3, 0), 0.5, false)).withTimeout(1.0), 
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false)),
            
            Commands.run(() -> drivebase.drive(new Translation2d(0, 0), 0, false)).withTimeout(1.0),
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false))
            
        );
    }

   
    
    public Command getRightAuto() {
        return Commands.sequence(
            Commands.run(() -> drivebase.drive(new Translation2d(0, 0), 0.5, false)).withTimeout(1.0), 
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false)),
            
            Commands.run(() -> drivebase.drive(new Translation2d(1, 0), 0, false)).withTimeout(1.0),
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false)),
            
            new ParallelCommandGroup(
                new AlignToReefNew(true, drivebase, () -> 0.0, () -> 0.0, () -> 0.0, null).withTimeout(10.0),
                Commands.runOnce(() -> elevator.engageStage(2)),

                Commands.sequence(
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(() -> elevator.setShooterSpeed(-1.0)),
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(() -> elevator.setShooterSpeed(0.0))
                )
            )
        );
    }
    
    public Command getLeftAuto() {
        return Commands.sequence(
            Commands.run(() -> drivebase.drive(new Translation2d(0, 0), -0.5, false)).withTimeout(1.0),
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false)),
            
            Commands.run(() -> drivebase.drive(new Translation2d(1.5, 0), 0, false)).withTimeout(1.6),
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false)),
            
            new ParallelCommandGroup(
                new AlignToReefNew(true, drivebase, () -> 0.0, () -> 0.0, () -> 0.0, null).withTimeout(10.0),
                Commands.sequence(
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(() -> elevator.setShooterSpeed(-1.0)),
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(() -> elevator.setShooterSpeed(0.0))
                )
            )
        );
    }
    
    public Command getMiddleAuto() {
        return Commands.sequence(
            
            
            new ParallelCommandGroup(
                new AlignToReefCenterAlgae(drivebase, () -> 0.0, () -> 0.0, () -> 0.0, elevator).withTimeout(5.0),
                Commands.sequence(
                    Commands.runOnce(() -> elevator.setAlgaeKickerSpeed(-1.0)), 
                    Commands.waitSeconds(10.0), 
                    Commands.runOnce(() -> elevator.setAlgaeKickerSpeed(0.0))   
                )
            ),
            
            Commands.run(() -> drivebase.drive(new Translation2d(-1.5, 0), 0, false)).withTimeout(0.7),
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false)),
            
            new ParallelCommandGroup(
                new AlignToReefNew(true, drivebase, () -> 0.0, () -> 0.0, () -> 0.0, null).withTimeout(10.0),
                Commands.sequence(
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(() -> elevator.setShooterSpeed(-1.0)),
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(() -> elevator.setShooterSpeed(0.0))
                )
            )
        );
    }
    public Command getAutonomousCommand(String autoName) {
        switch (autoName.toUpperCase()) {
            case "RIGHT":
            case "RIGHT AUTO":
                return getRightAuto();
            case "LEFT":
            case "LEFT AUTO":
                return getLeftAuto();
            case "MIDDLE":
            case "MIDDLE AUTO":
            case "CENTER":
                return getMiddleAuto();
            default:
                return getMiddleAuto();
        }
    }
}
