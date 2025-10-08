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
import java.io.File;
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.AlignToReefTagPose;

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
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(3, 5, 1, 4, 2); // Leader=3, Follower=5, Intake=1, Shooter=4, AlgaeKicker=2
  // Create command instances with proper dependency injection
  private final IncreaseCommand increaseCommand = new IncreaseCommand(elevator);
  private final DecreaseCommand decreaseCommand = new DecreaseCommand(elevator);
  

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1.0)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverXbox.getRightX() * -1,
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
                                                                         2) * -1)
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(1.0)
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
      
      // Driver controls (moved from operator)
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      
      // X button - FIXED: Algae kicking sequence with tag-based elevator positioning
      driverXbox.x().onTrue(
        new ParallelCommandGroup(
          // Align with algae-specific offset, prioritizing left limelight
          new AlignToReefTagRelative(false, drivebase, true, true), // false = center, true = force left camera, true = algae mode
          // Sequential elevator and algae kicker operations (no subsystem conflict)
          new SequentialCommandGroup(
            // Start algae kicker immediately
            Commands.runOnce(() -> elevator.startAlgaeKicker(), elevator),
            Commands.waitUntil(() -> {
              // Wait for alignment to start and detect a tag
              return !drivebase.getPose().equals(new Pose2d());
            }),
            // Move elevator to position based on detected tag
            Commands.runOnce(() -> {
              // Get the detected tag ID from the alignment command
              double detectedTagID = frc.robot.LimelightHelpers.getFiducialID("limelight-left");
              
              // If left camera doesn't see a tag, try right camera
              if (detectedTagID <= 0) {
                detectedTagID = frc.robot.LimelightHelpers.getFiducialID("limelight-right");
              }
              
              // Position A tags: 17, 11, 7, 21, 9, 19
              if (detectedTagID == 17 || detectedTagID == 11 || detectedTagID == 7 ||
                  detectedTagID == 21 || detectedTagID == 9 || detectedTagID == 19) {
                elevator.goToAlgaePositionA();
                System.out.println("ALGAE KICKING: Detected Position A tag " + detectedTagID + " - Moving to Algae Position A");
              }
              // Position B tags: 18, 7, 22, 6, 8, 20
              else if (detectedTagID == 18 || detectedTagID == 7 || detectedTagID == 22 ||
                       detectedTagID == 6 || detectedTagID == 8 || detectedTagID == 20) {
                elevator.goToAlgaePositionB();
                System.out.println("ALGAE KICKING: Detected Position B tag " + detectedTagID + " - Moving to Algae Position B");
              }
              // Unknown tag - default to Position A
              else {
                elevator.goToAlgaePositionA();
                System.out.println("ALGAE KICKING: Unknown/No tag detected (" + detectedTagID + ") - Defaulting to Algae Position A");
              }
              
              // Update SmartDashboard with tag detection info
              edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Algae Detected Tag ID", detectedTagID);
              edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Algae Elevator Position", 
                (detectedTagID == 17 || detectedTagID == 11 || detectedTagID == 7 ||
                 detectedTagID == 21 || detectedTagID == 9 || detectedTagID == 19) ? "Position A" :
                (detectedTagID == 18 || detectedTagID == 7 || detectedTagID == 22 ||
                 detectedTagID == 6 || detectedTagID == 8 || detectedTagID == 20) ? "Position B" : "Position A (Default)");
            }, elevator),
            // Wait for elevator to reach target position
            Commands.waitUntil(() -> {
              // Check if we're at either algae position
              return elevator.isElevatorAtAlgaePositionA() || elevator.isElevatorAtAlgaePositionB();
            })
          )
        )
        .finallyDo((interrupted) -> {
          // Stop algae kicker when command ends (whether completed or interrupted)
          elevator.stopAlgaeKicker();
        })
        .withName("X_AlgaeKick_TagBasedElevator_LeftCameraPriority")
      );
      
      // B button now controls shooter override in stages 1 and 2 with full power (30A limit)
      driverXbox.b()
        .whileTrue(new InstantCommand(() -> {
          elevator.setManualShooterOverride(true);
          // Set full power (-1.0) which will be current-limited to 30A by motor controller
          if (elevator.getStage() != 0) {
            elevator.setShooterSpeed(-1.0); // Full power in reverse direction
          }
        }, elevator))
        .onFalse(new InstantCommand(() -> {
          elevator.setManualShooterOverride(false);
          elevator.setShooterSpeed(0.0); // Stop shooter
        }, elevator));
      
      // Elevator stage controls on POV (D-pad)
      driverXbox.povUp().onTrue(increaseCommand);
      driverXbox.povDown().onTrue(decreaseCommand);
      
      // Intake control on left bumper - CHANGED TO STAGE 1 SEQUENCE
      driverXbox.leftBumper().onTrue(
        new ParallelCommandGroup(
          new AlignToReefTagRelative(false, drivebase), // LEFT alignment (false = left side)
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(1)), // Stage 1 instead of 2
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_ONE)),
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_ON)),
            Commands.waitSeconds(0.5), // Run shooter for 0.5 seconds
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_STOP))
          )
        ).withName("LeftBumper_LeftAlign_Stage1_Shooter")
      );

      // Right bumper - CHANGED TO STAGE 1 SEQUENCE  
      driverXbox.rightBumper().onTrue(
        new ParallelCommandGroup(
          new AlignToReefTagRelative(true, drivebase), // RIGHT alignment (true = right side)
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(1)), // Stage 1 instead of 2
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_ONE)),
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_ON)),
            Commands.waitSeconds(0.5), // Run shooter for 0.5 seconds
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_STOP))
          )
        ).withName("RightBumper_RightAlign_Stage1_Shooter")
      );

      // Reset resistance detection on start button
      driverXbox.start().onTrue(new InstantCommand(() -> elevator.resetResistanceDetection(), elevator));
      
      // Automated sequences on triggers with alignment - CHANGED TO PARALLEL
      driverXbox.leftTrigger().onTrue(
        new ParallelCommandGroup(
          new AlignToReefTagRelative(false, drivebase), // LEFT alignment (false = left side)
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(2)), // Stage 2
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_TWO)),
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_ON)),
            Commands.waitSeconds(0.5), // Run shooter for 0.5 seconds
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_STOP))
          )
        ).withName("LeftTrigger_LeftAlign_Stage2_Shooter")
      );
      
      driverXbox.rightTrigger().onTrue(
        new ParallelCommandGroup(
          new AlignToReefTagRelative(true, drivebase), // RIGHT alignment (true = right side)
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(2)), // Stage 2
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_TWO)),
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_ON)),
            Commands.waitSeconds(0.5), // Run shooter for 0.5 seconds
            Commands.runOnce(() -> elevator.setShooterSpeed(ElevatorConstants.SHOOTER_STOP))
          )
        ).withName("RightTrigger_RightAlign_Stage2_Shooter")
      );

      // Unused buttons for future expansion
      driverXbox.back().whileTrue(Commands.none());

      // Operator controller - all functions moved to driver controller
      // Keep operator available for additional functions if needed
      operatorXbox.a().whileTrue(Commands.none()); // Available for future use
      operatorXbox.b().whileTrue(Commands.none()); // Available for future use
      operatorXbox.x().whileTrue(Commands.none()); // Available for future use
      operatorXbox.y().whileTrue(Commands.none()); // Available for future use
     
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
            new AlignToReefTagPose(true, drivebase), // New pose-based alignment
            new InstantCommand(() -> elevator.engageStage(2))
        ),
        new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT))
    );
  }

  public Command reefTagIntakeStage1Command() {
    return new ParallelCommandGroup(
        new AlignToReefTagPose(true, drivebase), 
        new InstantCommand(() -> elevator.engageStage(1))
    )
    .andThen(new InstantCommand(() -> elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT))
    );
  }

}
