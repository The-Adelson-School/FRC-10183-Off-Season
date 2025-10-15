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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignToReefCenterAlgae;

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
import java.io.File;
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.AlignToReefNew;
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
  //teleop driver control
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1,
                                                                () -> driverXbox.getLeftX() * 1)
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
    
    // Elevator Position Commands for PathPlanner
    NamedCommands.registerCommand("STOWED_LEVEL", createStowedLevelCommand());
    NamedCommands.registerCommand("ALGAE_POSITION_A", createAlgaePositionACommand());
    NamedCommands.registerCommand("ALGAE_POSITION_B", createAlgaePositionBCommand());
    NamedCommands.registerCommand("LEVEL_ONE", createLevelOneCommand());
    NamedCommands.registerCommand("LEVEL_TWO", createLevelTwoCommand());
    NamedCommands.registerCommand("LEVEL_THREE", createLevelThreeCommand());
    
    // Shooter Commands for PathPlanner
//    NamedCommands.registerCommand("ALGAE_SHOOTER", createAlgaeShooterCommand());
    NamedCommands.registerCommand("SHOOTER", createShooterCommand());
    
    // SEPARATED: Individual Motor Control Commands for PathPlanner
    NamedCommands.registerCommand("START SHOOTER", createDirectStartShooterCommand());
    NamedCommands.registerCommand("STOP SHOOTER", createDirectStopShooterCommand());
    NamedCommands.registerCommand("RUN INTAKE 2 SECONDS", createRunIntake2SecondsCommand());
    
    // Legacy combined command (now just starts shooter)
    
    // Auto-Alignment Commands for PathPlanner
    NamedCommands.registerCommand("AUTO_ALIGN_LEFT", createAutoAlignLeftCommand());
    NamedCommands.registerCommand("AUTO_ALIGN_RIGHT", createAutoAlignRightCommand());
    
    System.out.println("PathPlanner Named Commands Registered:");
    System.out.println("- STOWED_LEVEL: Move elevator to stowed/home position");
    System.out.println("- ALGAE_POSITION_A: Move elevator to algae position A");
    System.out.println("- ALGAE_POSITION_B: Move elevator to algae position B");
    System.out.println("- LEVEL_ONE: Move elevator to level 1");
    System.out.println("- LEVEL_TWO: Move elevator to level 2");
    System.out.println("- LEVEL_THREE: Move elevator to level 3");
    System.out.println("- ALGAE_SHOOTER: Run algae kicker for 1 second");
    System.out.println("- SHOOTER: Run shooter for 1 second");
    System.out.println("- START SHOOTER: Start shooter only (STOWED level only)");
    System.out.println("- STOP SHOOTER: Stop shooter only");  
    System.out.println("- RUN INTAKE 2 SECONDS: Run intake for exactly 2 seconds (any level)");
    System.out.println("- START SHOOTER AND INTAKE: Legacy - same as START SHOOTER");
    System.out.println("- STOP SHOOTER AND INTAKE: Legacy - same as STOP SHOOTER");
    System.out.println("- AUTO_ALIGN_LEFT: Auto-align to AprilTag using left camera priority");
    System.out.println("- AUTO_ALIGN_RIGHT: Auto-align to AprilTag using right camera priority");
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
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (RobotBase.isSimulation())
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
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      //elevator.setDefaultCommand(new InstantCommand(() -> elevator.defaultCommand(), elevator));
      elevator.setDefaultCommand(
    Commands.run(() -> elevator.defaultCommand(), elevator)
);
      // Driver controls (moved from operator)
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      
      // X button - Dynamic algae autoalign with SWAPPED tag-based elevator positioning (RIGHT camera priority) - WITH DRIVER OVERRIDE
  // Build the command once so we can cancel it on button release
Command alignCenterAlgaeCmd = new AlignToReefCenterAlgae(
  drivebase,
  () -> driverXbox.getLeftY(),
  () -> driverXbox.getLeftX(),
  () -> driverXbox.getRightX(),
  elevator
).withName("X_AlignCenter_Algae");

// Start on press; command self-terminates on driver override.
// Also cancel on release if still running.
driverXbox.x()
.onTrue(alignCenterAlgaeCmd)
.onFalse(Commands.runOnce(alignCenterAlgaeCmd::cancel));

// Add alongside your existing X binding (keep that one unchanged)
driverXbox.x()
  .onTrue(Commands.runOnce(() -> elevator.startAlgaeKicker()))   // start on press
  .onFalse(Commands.runOnce(() -> elevator.stopAlgaeKicker()));  // stop on release

// Add alongside your existing X binding (keep that one unchanged)
//driverXbox.x().whileTrue(Commands.runOnce(() -> elevator.startAlgaeKicker())) ;  // start on press
 // .onFalse(Commands.runOnce(() -> elevator.stopAlgaeKicker()));  // stop on release


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
      
      // Intake control on left bumper - WITH DRIVER OVERRIDE AND AUTO SHOOTER (0.5 seconds after alignment)
      driverXbox.leftBumper().onTrue(
        new ParallelCommandGroup(
          new AlignToReefNew(false, drivebase,
                                   () -> driverXbox.getLeftY(), 
                                   () -> driverXbox.getLeftX(), 
                                   () -> driverXbox.getRightX(), 
                                   elevator), // CHANGED: Added elevator for auto-shooter after alignment
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(1)), // Stage 1
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_ONE))
          )
        ).withName("LeftBumper_LeftAlign_Stage1_AutoShooter_0.5s")
      );

      // Right bumper - WITH DRIVER OVERRIDE AND AUTO SHOOTER (0.5 seconds after alignment)
      driverXbox.rightBumper().onTrue(
        new ParallelCommandGroup(
          new AlignToReefNew(true, drivebase,
                                   () -> driverXbox.getLeftY(), 
                                   () -> driverXbox.getLeftX(), 
                                   () -> driverXbox.getRightX(), 
                                   elevator), // CHANGED: Added elevator for auto-shooter after alignment
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(1)), // Stage 1
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_ONE))
          )
        ).withName("RightBumper_RightAlign_Stage1_AutoShooter_0.5s")
      );

      // Reset resistance detection on start button
      driverXbox.start().onTrue(new InstantCommand(() -> elevator.resetResistanceDetection(), elevator));
      
      // Automated sequences on triggers with alignment - WITH DRIVER OVERRIDE AND AUTO SHOOTER (0.5 seconds after alignment)
      driverXbox.leftTrigger().onTrue(
        new ParallelCommandGroup(
          new AlignToReefNew(false, drivebase,
                                   () -> driverXbox.getLeftY(), 
                                   () -> driverXbox.getLeftX(), 
                                   () -> driverXbox.getRightX(), 
                                   elevator), // CHANGED: Added elevator for auto-shooter after alignment
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(2)), // Stage 2
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_TWO))
          )
        ).withName("LeftTrigger_LeftAlign_Stage2_AutoShooter_0.5s")
      );
      
      driverXbox.rightTrigger().onTrue(
        new ParallelCommandGroup(
          new AlignToReefNew(true, drivebase, 
                                   () -> driverXbox.getLeftY(), 
                                   () -> driverXbox.getLeftX(), 
                                   () -> driverXbox.getRightX(), 
                                   elevator), // CHANGED: Added elevator for auto-shooter after alignment
          new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.engageStage(2)), // Stage 2
            Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_TWO))
          )
        ).withName("RightTrigger_RightAlign_Stage2_AutoShooter_0.5s")
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
    // FIXED: Add comprehensive safety checks and initialization delays for autonomous
    return Commands.sequence(
      // Step 1: Ensure systems are fully ready
      Commands.runOnce(() -> System.out.println("AUTONOMOUS: Starting initialization checks...")),
      
      // Step 2: Wait for swerve drive to be fully initialized
      Commands.waitUntil(() -> {
        try {
          // Verify swerve drive is responsive
          drivebase.getPose();
          drivebase.getHeading();
          return true;
        } catch (Exception e) {
          System.err.println("AUTONOMOUS: Waiting for swerve drive initialization: " + e.getMessage());
          return false;
        }
      }).withTimeout(3.0), // Max 3 seconds wait
      
      // Step 3: Additional stabilization delay
      Commands.waitSeconds(0.5), // Allow systems to stabilize
      
      // Step 4: Reset any problematic states
      Commands.runOnce(() -> {
        try {
          // Ensure robot is in a known good state
          drivebase.drive(new ChassisSpeeds(0, 0, 0)); // Stop any movement
          System.out.println("AUTONOMOUS: Systems ready, starting path: MIDDLE AUTO TEST");
        } catch (Exception e) {
          System.err.println("AUTONOMOUS: Error in state reset: " + e.getMessage());
        }
      }),
      
      // Step 5: Run the actual autonomous command with error handling
      drivebase.getAutonomousCommand("RED RIGHT")
        .handleInterrupt(() -> {
          drivebase.drive(new ChassisSpeeds(0, 0, 0));
        })
        .andThen(Commands.runOnce(() -> {
          drivebase.drive(new ChassisSpeeds(0, 0, 0));
        }))
    )
    .handleInterrupt(() -> {

      drivebase.drive(new ChassisSpeeds(0, 0, 0));
    })
    .withName("SafeAutonomousCommand");
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

  // NEW: PathPlanner Named Command Factories

  /**
   * Move elevator to Algae Position A and wait until target is reached
   */
  private Command createAlgaePositionACommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.goToAlgaePositionA();
        System.out.println("PATHPLANNER: Moving elevator to Algae Position A (" + ElevatorConstants.ALGAE_POSITION_A + " counts)");
      }, elevator),
      Commands.waitUntil(() -> elevator.isElevatorAtAlgaePositionA()),
      Commands.runOnce(() -> System.out.println("PATHPLANNER: Elevator reached Algae Position A"))
    ).withName("AlgaePositionA");
  }

  /**
   * Move elevator to Algae Position B and wait until target is reached
   */
  private Command createAlgaePositionBCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.goToAlgaePositionB();
        System.out.println("PATHPLANNER: Moving elevator to Algae Position B (" + ElevatorConstants.ALGAE_POSITION_B + " counts)");
      }, elevator),
      Commands.waitUntil(() -> elevator.isElevatorAtAlgaePositionB()),
      Commands.runOnce(() -> System.out.println("PATHPLANNER: Elevator reached Algae Position B"))
    ).withName("AlgaePositionB");
  }

  /**
   * Move elevator to Level One and wait until target is reached
   */
  private Command createLevelOneCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.engageStage(1);
        System.out.println("PATHPLANNER: Moving elevator to Level One (" + ElevatorConstants.LEVEL_ONE + " counts)");
      }, elevator),
      Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_ONE)),
      Commands.runOnce(() -> System.out.println("PATHPLANNER: Elevator reached Level One"))
    ).withName("LevelOne");
  }

  /**
   * Move elevator to Level Two and wait until target is reached
   */
  private Command createLevelTwoCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.engageStage(2);
        System.out.println("PATHPLANNER: Moving elevator to Level Two (" + ElevatorConstants.LEVEL_TWO + " counts)");
      }, elevator),
      Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_TWO)),
      Commands.runOnce(() -> System.out.println("PATHPLANNER: Elevator reached Level Two"))
    ).withName("LevelTwo");
  }

  /**
   * Move elevator to Level Three and wait until target is reached
   */
  private Command createLevelThreeCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.engageStage(3);
        System.out.println("PATHPLANNER: Moving elevator to Level Three (" + ElevatorConstants.LEVEL_THREE + " counts)");
      }, elevator),
      Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.LEVEL_THREE)),
      Commands.runOnce(() -> System.out.println("PATHPLANNER: Elevator reached Level Three"))
    ).withName("LevelThree");
  }

  /**
   * Move elevator to Stowed Level (stage 0) and wait until target is reached
   */
  private Command createStowedLevelCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.engageStage(0);
        System.out.println("PATHPLANNER: Moving elevator to Stowed Level (" + ElevatorConstants.STOWED_LEVEL + " counts)");
      }, elevator),
      Commands.waitUntil(() -> elevator.isElevatorAtTarget(ElevatorConstants.STOWED_LEVEL)),
      Commands.runOnce(() -> System.out.println("PATHPLANNER: Elevator reached Stowed Level"))
    ).withName("StowedLevel");
  }

  /**
   * Run algae kicker for 1 second
   */
  /*
  private Command createAlgaeShooterCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.startAlgaeKicker();
      }, elevator),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> {
        elevator.stopAlgaeKicker();
      }, elevator)
    ).withName("AlgaeShooter1Sec");
  }
*/
  /**
   * Run shooter for 1 second
   */
  private Command createShooterCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.setShooterSpeed(-1.0); // Full power reverse
        System.out.println("PATHPLANNER: Starting shooter for 1 second");
      }, elevator),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> {
        elevator.setShooterSpeed(0.0); // Stop shooter
        System.out.println("PATHPLANNER: Shooter stopped after 1 second");
      }, elevator)
    ).withName("Shooter1Sec");
  }

  /**
   * Auto-align to AprilTag using left camera priority (for autonomous)
   */
  private Command createAutoAlignLeftCommand() {
    return new AlignToReefNew(
      false,           // isRightScore = false (left alignment)
      drivebase,       // SwerveSubsystem
      () -> 0.0,       // No driver override in autonomous
      () -> 0.0,       // No driver override in autonomous
      () -> 0.0,       // No driver override in autonomous
      null             // No elevator reference (alignment only, no shooter)
    ).withName("AutoAlignLeft").withTimeout(5.0); // 5 second timeout for safety
  }

  /**
   * Auto-align to AprilTag using right camera priority (for autonomous)
   */
  private Command createAutoAlignRightCommand() {
    return new AlignToReefNew(
      true,            // isRightScore = true (right alignment)
      drivebase,       // SwerveSubsystem
      () -> 0.0,       // No driver override in autonomous
      () -> 0.0,       // No driver override in autonomous
      () -> 0.0,       // No driver override in autonomous
      null             // No elevator reference (alignment only, no shooter)
    ).withName("AutoAlignRight").withTimeout(5.0); // 5 second timeout for safety
  }

  // SIMPLIFIED: Direct Motor Control Commands (No Manual Mode Required)

  /**
   * Start shooter and intake motors directly in autonomous
   * Safety check: Only works when robot is at STOWED LEVEL (stage 0)
   * Motors run continuously until stopped by STOP command
   */
  private Command createDirectStartShooterIntakeCommand() {
    return Commands.runOnce(() -> {
      if (elevator.isAtStowedLevel()) {
        // Direct motor control - start shooter at full reverse power
        elevator.setShooterSpeed(-1.0); // Full power reverse for shooting
        // Direct motor control - start intake outward
        elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT);

      }
    }, elevator).withName("DirectStartShooterIntake");
  }


  private Command createDirectStopShooterIntakeCommand() {
    return Commands.runOnce(() -> {
      elevator.setShooterSpeed(0.0);
      elevator.setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
    }, elevator).withName("DirectStopShooterIntake");
  }

  // SEPARATED: Individual Motor Control Commands

  /**
   * Start ONLY shooter motor directly in autonomous
   * Safety check: Only works when robot is at STOWED LEVEL (stage 0)
   * NO INTAKE - shooter runs continuously until stopped
   */
  private Command createDirectStartShooterCommand() {
    return Commands.runOnce(() -> {
      if (elevator.isAtStowedLevel()) {
        elevator.setShooterSpeed(-1.0); // Full power reverse for shooting
        System.out.println("PATHPLANNER: Starting SHOOTER ONLY at stowed level");
      } else {
        System.out.println("PATHPLANNER ERROR: Cannot start shooter - robot not at STOWED LEVEL");
      }
    }, elevator).withName("DirectStartShooter");
  }

  /**
   * Stop ONLY shooter motor directly in autonomous
   * Works from any elevator level
   */
  private Command createDirectStopShooterCommand() {
    return Commands.runOnce(() -> {
      elevator.setShooterSpeed(0.0);
      System.out.println("PATHPLANNER: Stopping SHOOTER");
    }, elevator).withName("DirectStopShooter");
  }

  /**
   * Run intake motor for EXACTLY 2 seconds then automatically stop
   * Works from any elevator level - NO restrictions
   * This is the ONLY way intake runs in autonomous unless manually controlled
   */
  private Command createRunIntake2SecondsCommand() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        elevator.setIntakeSpeed(ElevatorConstants.INTAKE_OUT);
        System.out.println("PATHPLANNER: Starting intake for 2 seconds");
      }, elevator),
      Commands.waitSeconds(2.0),
      Commands.runOnce(() -> {
        elevator.setIntakeSpeed(ElevatorConstants.INTAKE_STOP);
        System.out.println("PATHPLANNER: Intake stopped after 2 seconds");
      }, elevator)
    ).withName("RunIntake2Seconds");
  }

  // LEGACY: Keep old combined commands for backward compatibility (but they only affect shooter now)
  //private Command createDirectStartShooterIntakeCommand() {
  //  return createDirectStartShooterCommand(); // Just start shooter, NO intake
  //}

  //private Command createDirectStopShooterIntakeCommand() {
   // return createDirectStopShooterCommand(); // Just stop shooter, intake handled separately
 // }

}


