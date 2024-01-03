// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ChaseAprilTagCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OperatorCommand;
import frc.robot.commands.ResetIntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Lights.LightState;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.*;

import java.io.File;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
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
    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
    private final Intake intake = new Intake(driverXbox.getHID());
    public static final Lights lights = new Lights();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();

    private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());
    private final OperatorCommand normalOperator = new OperatorCommand(intake, operatorXbox.getHID());

    // private final TagFollowCommand tagFollow = new
    // TagFollowCommand(swerveDriveSubsystem, driverXbox.getHID());

    DigitalInput intakeLimitSw = new DigitalInput(IntakeConstants.LIMIT_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        File ppdir = Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner").toFile();
        int i = 0;
        for (String file : ppdir.list()) {
            if (file.endsWith(".path")) {
                System.out.println(file);
                String fname = file.replace(".path", "");
                if (i == 0)
                    Robot.autoChooser.setDefaultOption(fname, fname);
                Robot.autoChooser.addOption(fname, fname);
                i++;
            }
        }

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
    private Command resetCommand = new ParallelRaceGroup(new WaitCommand(10.0),
            new ResetIntakeCommand(intake, intakeLimitSw));

    private Command otfCommandRunning = null;
    private Pose2d target = null;

    private void configureBindings() {
        // Needs to be held for 1/2 second!
        // Automatically aborted if the intake takes longer than 10 seconds to zero
        // (probably jammed in that case)
        operatorXbox.y().debounce(0.5).onTrue(resetCommand);
        operatorXbox.a().onTrue(new InstantCommand(() -> resetCommand.cancel()));

        // Use Y to enable tag following!
        //driverXbox.y().onTrue(tagFollow).onFalse(new InstantCommand(() ->
        // tagFollow.cancel()));

        // use b to chase April tag
        driverXbox.b().whileTrue(new ChaseAprilTagCommand(swerveDriveSubsystem));

        // Drive with a on-the-fly generated path to (0,0) WHILE right bumper is held
        driverXbox.rightBumper().and(new BooleanSupplier() {
            public boolean getAsBoolean() {
                return target != null;
            };
        }).onTrue(new InstantCommand(() -> {
            // Instant command so path is generated at the time the trigger is pressed
            // (dynamically)
            otfCommandRunning = genOTFDriveCommand(true, target);
            otfCommandRunning.schedule();
        })).onFalse(new InstantCommand(() -> {
            otfCommandRunning.cancel();
        }));

        driverXbox.y().and(new BooleanSupplier() {
            public boolean getAsBoolean() {
                return target != null;
            };
        }).onTrue(new InstantCommand(() -> {
            // Instant command so path is generated at the time the trigger is pressed
            // (dynamically)
            otfCommandRunning = genOTFDriveCommand(true,
                    new Pose2d(target.getTranslation().plus(new Translation2d(-1.5, target.getRotation())),
                            target.getRotation()),
                    target);
            otfCommandRunning.schedule();
        })).onFalse(new InstantCommand(() -> {
            otfCommandRunning.cancel();
        }));

        driverXbox.a().onTrue(new InstantCommand(() -> {
            target = swerveDriveSubsystem.getPose();
        }));

        intake.intakeStallTrigger().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            driverXbox.getHID().setRumble(RumbleType.kBothRumble, 1);
            operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 1);
        }))).onFalse(new InstantCommand(() -> {
            driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
            operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
        }));
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

        // TODO: FIX AUTO!

        // CommandBase shootcommand = new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // System.out.println("Starting Shooting");
        // intake.setIntakeState(IntakeState.HIGH);
        // }),
        // new WaitCommand(0.75),
        // new InstantCommand(() -> {
        // intake.enableCurrentControl(false);
        // intake.setIntakeSpeed(1.0);
        // }),
        // new WaitCommand(0.5),
        // new InstantCommand(() -> {
        // intake.setIntakeSpeed(0.0);
        // intake.enableCurrentControl(true);
        // intake.setIntakeState(IntakeState.STOWED);
        // System.out.println("Done Shooting");
        // }));

        // shootcommand.addRequirements(intake);

        // return shootcommand;

        intake.setIntakeState(IntakeState.STOWED);
        List<PathPlannerTrajectory> traj = PathPlanner.loadPathGroup(Robot.autoChooser.getSelected(),
                new PathConstraints(Constants.DriveConstants.MAX_ROBOT_VELOCITY / 1.5,
                        Constants.DriveConstants.MAX_ROBOT_VELOCITY / 1.5));

        /*
         * new PathConstraints(Constants.DriveConstants.MAX_ROBOT_VELOCITY / 1.5,
         * Constants.DriveConstants.MAX_ROBOT_VELOCITY / 1.5)
         */

        return getAutoCommand(traj);
        //return new ChaseAprilTagCommand(swerveDriveSubsystem, targetX, targetY, targetAngle, xSpeedWidget, ySpeedWidget, omegaSpeedWidget, tagErrorWidget);
    }

    public Command getAutoCommand(List<PathPlannerTrajectory> path) {

        SequentialCommandGroup auton = new SequentialCommandGroup();
        auton.addCommands(new InstantCommand(() -> lights.setState(LightState.LOGO)));

        // Pre-drive commands
        if (path.size() > 0 && path.get(0).getStartStopEvent().names.size() > 0) {
            CommandBase intakecommand = new InstantCommand(() -> {
                System.out.println("Starting intake!");
                intake.setIntakeState(IntakeState.PICKUP);
                intake.addIntakeState(-10.0);
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
                        intake.setIntakeState(IntakeState.HIGH);
                    }),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> {
                        intake.enableCurrentControl(false);
                        intake.setIntakeSpeed(1.0);
                    }),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> {
                        intake.setIntakeSpeed(0.0);
                        intake.enableCurrentControl(true);
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
            for (String name : path.get(0).getStartStopEvent().names) {
                if (eventMap.containsKey(name))
                    auton.addCommands(eventMap.get(name));
            }
        }

        for (int i = 0; i < path.size(); ++i) {
            CommandBase intakecommand = new InstantCommand(() -> {
                System.out.println("Starting intake!");
                intake.setIntakeState(IntakeState.PICKUP);
                intake.addIntakeState(-10.0);
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
                        intake.setIntakeState(IntakeState.HIGH);
                    }),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> {
                        intake.enableCurrentControl(false);
                        intake.setIntakeSpeed(1.0);
                    }),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> {
                        intake.setIntakeSpeed(0.0);
                        intake.enableCurrentControl(true);
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
            auton.addCommands(followTrajectoryCommand(path.get(i), i == 0, eventMap, true));
            List<String> names = path.get(i).getEndStopEvent().names;

            // if (names.size() > 0)
            // auton.addCommands(stopcommand);

            for (String name : names) {
                if (eventMap.containsKey(name))
                    auton.addCommands(eventMap.get(name));
            }
        }

        auton.addCommands(new InstantCommand(() -> {
            // swerveDriveSubsystem.setHeading(180);
            // swerveDriveSubsystem.setHeading(path.get(path.size()-1).getEndState().holonomicRotation.getDegrees());
        }));

        return auton;
    }

    // Assuming this method is part of a drivetrain subsystem that provides the
    // necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath,
            HashMap<String, Command> eventMap, boolean mirror) {

        PathPlannerTrajectory t_transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj,
                mirror ? DriverStation.getAlliance() : Alliance.Blue);

        Command swerveController = new PPSwerveControllerCommand(
                t_transformed,
                swerveDriveSubsystem::getPose, // Pose supplier
                new PIDController(
                        5.0,
                        0,
                        0), // X controller
                new PIDController(
                        5.0,
                        0,
                        0), // Y controller
                new PIDController(2.0, 0, 0.0), // Rotation controller
                swerveDriveSubsystem::setChassisSpeedsAUTO, // Chassis speeds states consumer
                false, // Should the path be automatically mirrored depending on alliance color.
                       // Optional, defaults to true
                swerveDriveSubsystem // Requires this drive subsystem
        );

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        SmartDashboard.putString("DSA", DriverStation.getAlliance().toString());
                        swerveDriveSubsystem.zeroHeading();
                        Pose2d initpose = t_transformed.getInitialHolonomicPose();
                        swerveDriveSubsystem.setHeading(initpose.getRotation().getDegrees());
                        swerveDriveSubsystem.resetOdometry(initpose);
                    }
                }),
                new FollowPathWithEvents(swerveController, t_transformed.getMarkers(), eventMap));
    }

    public Command genOTFDriveCommand(boolean useRotation, Pose2d... points) {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (Pose2d pose : points) {
            poses.add(pose);
        }
        return genOTFDriveCommand(useRotation, poses);
    }

    public Command genOTFDriveCommand(boolean useRotation, List<Pose2d> points) {
        List<PathPlannerTrajectory.EventMarker> evts = new ArrayList<>();
        PathPlannerTrajectory.EventMarker startmarker = new PathPlannerTrajectory.EventMarker(List.of("start"), 0.0);
        PathPlannerTrajectory.EventMarker endmarker = new PathPlannerTrajectory.EventMarker(List.of("end"),
                points.size() * 1.0);
        evts.add(endmarker);
        evts.add(startmarker);
        Rotation2d driveangle_init = points.get(0).getTranslation()
                .minus(swerveDriveSubsystem.getPose().getTranslation()).getAngle();

        List<PathPoint> pp = new ArrayList<>();
        double speed = new Translation2d(swerveDriveSubsystem.getChassisSpeeds().vxMetersPerSecond,
                swerveDriveSubsystem.getChassisSpeeds().vyMetersPerSecond).getNorm();
        pp.add(new PathPoint(swerveDriveSubsystem.getPose().getTranslation(), driveangle_init,
                swerveDriveSubsystem.getPose().getRotation(), speed));

        for (int i = 0; i < points.size(); i++) {
            Rotation2d driveangle = points.get(i).getTranslation().minus(pp.get(i).position).getAngle();
            pp.add(new PathPoint(points.get(i).getTranslation(), driveangle,
                    useRotation ? points.get(i).getRotation() : swerveDriveSubsystem.getPose().getRotation()));
            System.out.println(points.get(i).getRotation().toString());
        }

        PathPlannerTrajectory hometraj = PathPlanner.generatePath(
                new PathConstraints(Constants.DriveConstants.MAX_ROBOT_VELOCITY / 1.5,
                        Constants.DriveConstants.MAX_ROBOT_VELOCITY / 1.5),
                // new PathConstraints(Constants.DriveConstants.MAX_ROBOT_VELOCITY/2.0,
                // Constants.DriveConstants.MAX_ROBOT_VELOCITY/3.0),
                pp,
                evts);

        Map<String, Command> evtmap = Map.of(
                "start", ((Command) new PrintCommand("############# Starting OTF path!")),
                "end", ((Command) new PrintCommand("///////////// Finished OTF path!")));

        CommandBase followcmd = (CommandBase) followTrajectoryCommand(hometraj, false, new HashMap<>(evtmap), false);
        return followcmd;
    }
}
