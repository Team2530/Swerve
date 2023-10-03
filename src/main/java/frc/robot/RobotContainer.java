// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.*;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    swerveDriveSubsystem.setDefaultCommand(normalDrive);
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
    PathPlannerTrajectory traj = PathPlanner.loadPath("Spin",
        new PathConstraints(Constants.DriveConstants.MAX_ROBOT_VELOCITY / 2.0,
            Constants.DriveConstants.MAX_ROBOT_VELOCITY / 2.0));

    return followTrajectoryCommand(traj, true);
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
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

        // TODO: TUNE!!!!
        new PPSwerveControllerCommand(
            traj,
            swerveDriveSubsystem::getPose, // Pose supplier
            new PIDController(
                1.0,
                0,
                0), // X controller
            new PIDController(
                1.0,
                0,
                0), // Y controller
            new PIDController(1.0, 0, 0), // Rotation controller
            swerveDriveSubsystem::setChassisSpeedsAUTO, // Chassis speeds states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            swerveDriveSubsystem // Requires this drive subsystem
        ));
  }

}
