// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.subsystems.elevator.AutoAlignWrapper;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.AutoAlignWrapper;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.hanger.HangerSubsystem;
import frc.robot.subsystems.AlignToReefTagRelative;
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
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(6, 8, 8, 8, 8);
  private final HangerSubsystem hanger = new HangerSubsystem(9);
  
  

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
    NamedCommands.registerCommand("Intake Out", new InstantCommand(() -> elevator.autonomousCommand()));
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
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

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
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  //    driverXbox.rightBumper().onTrue(Commands.none());
      //Operator Bindings
      operatorXbox.povUp().onTrue(new InstantCommand(() -> elevator.increaseStage(), elevator));
      operatorXbox.povDown().onTrue(new InstantCommand(() -> elevator.decreaseStage(), elevator));
    //  operatorXbox.rightBumper().whileTrue(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN), elevator));
    //  operatorXbox.leftBumper().whileTrue(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT), elevator));
      // Right bumper: Spins intake inward while held
operatorXbox.rightBumper()
.whileTrue(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN), elevator))
.onFalse(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_STOP), elevator));

// Left bumper: Spins intake outward while held
operatorXbox.leftBumper()
.whileTrue(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT), elevator))
.onFalse(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_STOP), elevator));
    //Hanger Stuff
    operatorXbox.y().onTrue(new InstantCommand(() -> hanger.setStage(1), hanger));
    operatorXbox.x().onTrue(new InstantCommand(() -> hanger.setStage(0), hanger));
    operatorXbox.a().onTrue(new InstantCommand(() -> hanger.setStage(2), hanger));
    //Camera Stuff
    driverXbox.rightBumper().onTrue(
        new SequentialCommandGroup(
            // Align with the reef tag
            new ParallelCommandGroup(
            new AlignToReefTagRelative(true, drivebase),
            new InstantCommand(() -> elevator.engageStage(4))
            ),
            // Set intake motor to output
            new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT))
        )
    );
    driverXbox.a().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.engageStage(2)),
        new WaitCommand(6),
        new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT))
      )
    );

    driverXbox.leftBumper().onTrue(
    new SequentialCommandGroup(
        // Run AlignToReefTagRelative and engageStage(4) simultaneously
        new ParallelCommandGroup(
            new AlignToReefTagRelative(true, drivebase),
            new InstantCommand(() -> elevator.engageStage(3))
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
            new AlignToReefTagRelative(true, drivebase),
            new InstantCommand(() -> elevator.engageStage(4))
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
      new AlignToReefTagRelative(true, drivebase),
      new InstantCommand(() -> elevator.engageStage(1))
  )
  .andThen(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN)))
  .andThen(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_IN)));
}


}
