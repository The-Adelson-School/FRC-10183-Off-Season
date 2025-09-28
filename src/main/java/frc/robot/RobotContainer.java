// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.
// THIS SHOULD BE ON FRC OFF SEASON
package frc.robot; 
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IncreaseCommand;
import frc.robot.commands.DecreaseCommand;
import frc.robot.subsystems.elevator.AutoAlignWrapper;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlignToReefTagRelative;
import frc.robot.AutoMovements; // Use the main AutoMovements class
import java.io.File;
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.NamedCommands;

  // Other setup code...

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer
{
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(6, 7, 8, 9, 0); // Leader=6, Follower=7, Intake=8, Shooter=9
  private final AutoMovements autoMovements = new AutoMovements(drivebase); // Use main AutoMovements class
  private final ClosestMovement closestMovement = new ClosestMovement(autoMovements, drivebase);
  
  // Create command instances with proper dependency injection
  private final IncreaseCommand increaseCommand = new IncreaseCommand(elevator);
  private final DecreaseCommand decreaseCommand = new DecreaseCommand(elevator);
  

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Use the proper non-blocking autonomous command
    NamedCommands.registerCommand("Intake Out", elevator.createAutonomousCommand());
    NamedCommands.registerCommand("test", Commands.print("I EXIST!"));
    NamedCommands.registerCommand("ReefTagIntake", reefTagIntakeSequence());
    NamedCommands.registerCommand("TestCommand", new InstantCommand(() -> System.out.println("Test!")));
    NamedCommands.registerCommand("ReefTagIntakeStage1", reefTagIntakeStage1Command());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

     if (Robot.isSimulation())
    {
      driveDirectAngleKeyboard.driveToPose(() -> new Pose2d(new Translation2d(9, 3),
                                                            Rotation2d.fromDegrees(90)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5,
                                                                                     3)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(
                                                                         Math.toRadians(
                                                                             360),
                                                                         Math.toRadians(
                                                                             90))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.print("Drive to pose simulation test"));

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      elevator.setDefaultCommand(new InstantCommand(() -> elevator.defaultCommand(), elevator));
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(closestMovement.moveToClosestRightPosition());
      driverXbox.y().onTrue(closestMovement.moveToClosestLeftPosition());
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      //Operator Bindings - Use proper commands with dependency injection
      operatorXbox.povUp().onTrue(increaseCommand);
      operatorXbox.povDown().onTrue(decreaseCommand);

      // Right bumper: Manually override intake inward while held (only works if not in stage 0)
      operatorXbox.rightBumper()
        .whileTrue(new InstantCommand(() -> {
          if (elevator.getStage() != 0) {  // Only allow manual control when not in stage 0
            elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN);
          }
        }, elevator))
        .onFalse(new InstantCommand(() -> {
          if (elevator.getStage() != 0) {  // Only reset when not in stage 0
            elevator.setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
          }
        }, elevator));

      // Left bumper: Manually override intake outward while held (works in any stage)
      operatorXbox.leftBumper()
        .whileTrue(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT), elevator))
        .onFalse(new InstantCommand(() -> {
          // When released, restore automatic intake behavior based on stage
          if (elevator.getStage() == 0) {
            elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN);  // Auto-intake in stage 0
          } else {
            elevator.setIntakeSpeed(ElevatorConstants.INTAKE_STOP); // Stop in other stages
          }
        }, elevator));

      // Add manual shooter control (X button for operator)
      operatorXbox.x()
        .whileTrue(new InstantCommand(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_ON), elevator))
        .onFalse(new InstantCommand(() -> {
          // When released, restore automatic shooter behavior based on stage
          if (elevator.getStage() == 0) {
            elevator.setShooterSpeed(ElevatorConstants.SHOOTER_ON);  // Auto-shooter in stage 0
          } else {
            elevator.setShooterSpeed(ElevatorConstants.SHOOTER_STOP); // Stop in other stages
          }
        }, elevator));

      // Add resistance reset button (B button for operator)
      operatorXbox.b().onTrue(new InstantCommand(() -> elevator.resetResistanceDetection(), elevator));

      //Camera Stuff - Right bumper for original limelight
      driverXbox.rightBumper().onTrue(
          new SequentialCommandGroup(
              // Align with the reef tag using original limelight
              new ParallelCommandGroup(
              new AlignToReefTagRelative(true, drivebase, false),
              new InstantCommand(() -> elevator.engageStage(2))  // Changed from 4 to 2
              ),
              // Set intake motor to output
              new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT))
          )
      );
      
      // Left bumper for second limelight (opposite side)
      driverXbox.leftBumper().onTrue(
      new SequentialCommandGroup(
          // Run AlignToReefTagRelative with second limelight and engageStage(2) simultaneously
          new ParallelCommandGroup(
              new AlignToReefTagRelative(true, drivebase, true),
              new InstantCommand(() -> elevator.engageStage(2))  // Changed from 3 to 2
          ),
          new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT))
      )
      );

      driverXbox.leftTrigger().onTrue(
      new ParallelCommandGroup(
        new AlignToReefTagRelative(true, drivebase),
        new InstantCommand(() -> elevator.engageStage(1))
      )
      .andThen(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN)))
      .andThen(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN)))
      );
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("TEST1");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  public Command reefTagIntakeSequence() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new AlignToReefTagRelative(true, drivebase, false),
            new InstantCommand(() -> elevator.engageStage(2))  // Changed from 4 to 2
        ),
        new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT))
    );
}
private void registerNamedCommands() {
  NamedCommands.registerCommand("ReefTagIntake", reefTagIntakeSequence());
  // Add more named commands here
}
public Command reefTagIntakeStage1Command() {
  return new ParallelCommandGroup(
      new AlignToReefTagRelative(true, drivebase, false),
      new InstantCommand(() -> elevator.engageStage(1))
  )
  .andThen(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN)))
  .andThen(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN)));
}


}
