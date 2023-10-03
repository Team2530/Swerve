// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake.IntakeState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.*;

import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController driverXbox = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final XboxController operatorXbox = new XboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
  private final Intake intake = new Intake(driverXbox);

  private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox);
  private final OperatorCommand normalOperator = new OperatorCommand(intake, driverXbox);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    swerveDriveSubsystem.setDefaultCommand(normalDrive);
    intake.setDefaultCommand(normalOperator);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // PathPlannerTrajectory traj = PathPlanner.generatePath(
    // new PathConstraints(Constants.DriveConstants.MAX_ROBOT_VELOCITY, 3),
    // new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), //
    // position, heading
    // new PathPoint(new Translation2d(10.0, 0), Rotation2d.fromDegrees(0)) //
    // position, heading
    // );
    List<PathPlannerTrajectory> traj = PathPlanner.loadPathGroup("TestPath",
        new PathConstraints(Constants.DriveConstants.MAX_ROBOT_VELOCITY / 2.0,
            Constants.DriveConstants.MAX_ROBOT_VELOCITY / 2.0));

    return getAutoCommand(traj);
  }

  public Command getAutoCommand(List<PathPlannerTrajectory> path) {
    CommandBase intakecommand = new InstantCommand(() -> {
      System.out.println("Starting intake!");
      intake.setIntakeState(IntakeState.PICKUP);
      intake.setIntakeSpeed(-0.2);
    });
    CommandBase stowcommand = new InstantCommand(() -> {
      System.out.println("Stowing intake!");
      intake.setIntakeState(IntakeState.STOWED);
      intake.setIntakeSpeed(-0.2);
    });
    CommandBase shootcommand = new SequentialCommandGroup(
        new InstantCommand(() -> {
          System.out.println("Starting Shooting");
          intake.setIntakeState(IntakeState.PLACE);
        }),
        new WaitCommand(0.5),
        new InstantCommand(() -> {
          intake.setIntakeSpeed(1.0);
        }),
        new WaitCommand(1.0),
        new InstantCommand(() -> {
          intake.setIntakeSpeed(0.0);
          intake.setIntakeState(IntakeState.STOWED);
          System.out.println("Done Shooting");
        }));
    intakecommand.addRequirements(intake);
    stowcommand.addRequirements(intake);
    shootcommand.addRequirements(intake);

    CommandBase stopcommand = new InstantCommand(() -> {
      swerveDriveSubsystem.stopDrive();
    });
    stopcommand.addRequirements(swerveDriveSubsystem);

    HashMap<String, Command> eventMap = new HashMap<>();
    // Make the intake intake
    eventMap.put("intake", intakecommand);
    // Stow the intake, hold game object
    eventMap.put("stow", stowcommand);
    // Rotate to shoot, shoot at max power
    eventMap.put("shoot", shootcommand);

    SequentialCommandGroup auton = new SequentialCommandGroup();

    for (int i = 0; i < path.size(); ++i) {
      // if (i != 0) {
      // auton.addCommands(new SequentialCommandGroup(
      // new WaitCommand(1.0),
      // new InstantCommand(() -> {
      // System.out.println("Between paths");
      // }),
      // new WaitCommand(1.0)));
      // }
      auton.addCommands(followTrajectoryCommand(path.get(i), i == 0, eventMap));
      List<String> names = path.get(i).getEndStopEvent().names;

      // if (names.size() > 0)
      // auton.addCommands(stopcommand);

      for (String name : names) {
        if (eventMap.containsKey(name))
          auton.addCommands(eventMap.get(name));
      }
    }

    return auton;
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath,
      HashMap<String, Command> eventMap) {
    Command swerveController = new PPSwerveControllerCommand(
        traj,
        swerveDriveSubsystem::getPose, // Pose supplier
        new PIDController(
            3.0,
            0,
            0), // X controller
        new PIDController(
            3.0,
            0,
            0), // Y controller
        new PIDController(0.3, 0, 0.0), // Rotation controller
        swerveDriveSubsystem::setChassisSpeedsAUTO, // Chassis speeds states consumer
        true, // Should the path be automatically mirrored depending on alliance color.
              // Optional, defaults to true
        swerveDriveSubsystem // Requires this drive subsystem
    );

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            swerveDriveSubsystem.zeroHeading();
            Pose2d initpose = traj.getInitialHolonomicPose();
            swerveDriveSubsystem.resetOdometry(
                new Pose2d(new Translation2d(initpose.getX(), -initpose.getY()), initpose.getRotation()));
          }
        }),

        new FollowPathWithEvents(swerveController, traj.getMarkers(), eventMap));
  }
}
