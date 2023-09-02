package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {
    SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_RADIANS,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED);

    SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_RADIANS,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED);

    SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_RADIANS,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED);

    SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_RADIANS,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, getRotation2d(),
            getModulePositions());

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
                // TODO: handle exception
            }
        }).start();
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());

        SmartDashboard.putString("Robot Pose", getPose().getTranslation().toString());
    }

    public void zeroHeading() {
        navX.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getHeading() {
        return Math.IEEEremainder(navX.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    public void stopDrive() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] states) {
        // Normalize speeds so they are all obtainable
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_MODULE_VELOCITY);
        frontLeft.setModuleState(states[0]);
        frontRight.setModuleState(states[1]);
        backLeft.setModuleState(states[2]);
        backRight.setModuleState(states[3]);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(frontLeft.getModuleState(), frontRight.getModuleState(),
                backLeft.getModuleState(), backRight.getModuleState());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        states[0] = frontLeft.getModulePosition();
        states[1] = frontRight.getModulePosition();
        states[2] = backLeft.getModulePosition();
        states[3] = backRight.getModulePosition();

        return states;
    }
}
