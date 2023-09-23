// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
  private final Joystick joystick = new Joystick(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final XboxController xbox = new XboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
  private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, joystick, xbox);

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
    return followTrajectoryCommand(PathPlanner.loadPath("TestPath", new PathConstraints(0.5, 1)), true);
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            swerveDriveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj,
            swerveDriveSubsystem::getPose, // Pose supplier
            DriveConstants.KINEMATICS, // SwerveDriveKinematics
            new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
                                        // feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
                                        // only use feedforwards.
            swerveDriveSubsystem::setModules, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            swerveDriveSubsystem // Requires this drive subsystem
        ));
  }

}
