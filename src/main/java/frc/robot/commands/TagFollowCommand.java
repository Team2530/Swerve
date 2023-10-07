package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.SwerveSubsystem;

public class TagFollowCommand extends CommandBase {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 1;
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(-1.5, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI));

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    public TagFollowCommand(
            SwerveSubsystem swerveSubsystem,
            XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = swerveSubsystem.getPose();

        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        if (xbox.getYButton()) {
            LimelightResults results = LimelightHelpers.getLatestResults(null);
            LimelightTarget_Fiducial target = null;
            for (LimelightTarget_Fiducial fid : results.targetingResults.targets_Fiducials) {
                if (fid.fiducialID == TAG_TO_CHASE) {
                    target = fid;
                    break;
                }
            }

            Pose3d robotPose = new Pose3d(swerveSubsystem.getPose());

            if (target == null) {
                swerveSubsystem.stopDrive();
            } else {
                // Tag in bot space
                Pose3d targetPose = target.getTargetPose_RobotSpace();
                Pose2d targetPose2d = new Pose2d(new Translation2d(targetPose.getZ(), -targetPose.getX()),
                        targetPose.getRotation().toRotation2d());

                // Tag and offset in field space
                var goalPose = new Pose3d(targetPose2d).relativeTo(robotPose).transformBy(TAG_TO_GOAL).toPose2d();

                // Drive
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                omegaController.setGoal(goalPose.getRotation().getRadians());

                // Drive to the target
                var xSpeed = xController.calculate(robotPose.getX());
                if (xController.atGoal()) {
                    xSpeed = 0;
                }

                var ySpeed = yController.calculate(targetPose2d.getY());
                if (yController.atGoal()) {
                    ySpeed = 0;
                }

                var omegaSpeed = omegaController.calculate(robotPose.toPose2d().getRotation().getRadians());
                if (omegaController.atGoal()) {
                    omegaSpeed = 0;
                }

                System.out.println(goalPose.getTranslation().toString());
                // swerveSubsystem.drive(xSpeed, ySpeed, omegaSpeed, true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !xbox.getYButton();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

}