package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Joystick joystick;

    public DriveCommand(SwerveSubsystem swerveSubsystem, Joystick joystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.joystick = joystick;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = joystick.getX();
        double ySpeed = joystick.getY();
        double zSpeed = joystick.getZ();

        xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.05 ? ySpeed : 0.0;
        zSpeed = Math.abs(zSpeed) > 0.05 ? zSpeed : 0.0;

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        ChassisSpeeds speeds;

        // Drive Field Oriented
        if (joystick.getRawButton(2)) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveSubsystem.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        }

        SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        swerveSubsystem.setModules(calculatedModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
