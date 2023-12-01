package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
    SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_ROTATIONS,
    SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED,
    SwerveModuleConstants.FL_DRIVE_MOTOR_REVERSED, SwerveModuleConstants.FL_STEER_MOTOR_REVERSED);

    private final SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
        SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_ROTATIONS,
        SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED,
        SwerveModuleConstants.FR_DRIVE_MOTOR_REVERSED, SwerveModuleConstants.FR_STEER_MOTOR_REVERSED);

    private final SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
        SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_ROTATIONS,
        SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED,
        SwerveModuleConstants.BR_DRIVE_MOTOR_REVERSED, SwerveModuleConstants.BR_STEER_MOTOR_REVERSED);

    private final SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
        SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_ROTATIONS,
        SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED,
        SwerveModuleConstants.BL_DRIVE_MOTOR_REVERSED, SwerveModuleConstants.BL_STEER_MOTOR_REVERSED);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private final Field2d field = new Field2d();

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, getRotation2d(),
            getModulePositions());

    public SwerveSubsystem() {
        zeroHeading();
        // new Thread(() -> {
        // try {
        // Thread.sleep(1000);
        // zeroHeading();
        // } catch (Exception e) {
        // // TODO: handle exception
        // }
        // }).start();
    }

    @Override
    public void periodic() {
        double rads = getPose().getRotation().getRadians(); 
        odometry.update(getRotation2d(), getModulePositions());
        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);

        SmartDashboard.putString("Robot Pose",
                getPose().toString());

        SmartDashboard.putNumber("Spin Velocity", (getPose().getRotation().getRadians() - rads) / 0.02);
    }

    public void zeroHeading() {
        // Reset pose to the same translation but rotation of 0
        var pose = odometry.getPoseMeters();
        var newPose = new Pose2d(pose.getTranslation(), new Rotation2d());
        navX.reset();
        resetOdometry(newPose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Gets the robot heading.
     * @return robot heading in radians [-Pi, Pi) with counter-clockwise positive
     */
    public double getHeading() {
        return -Units.degreesToRadians(Math.IEEEremainder(navX.getAngle(), 360d));
    }

    /**
     * Gets the robot heading.
     * @return robot heading in radians [-Pi, Pi) with counter-clockwise positive
     */
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
        backRight.setModuleState(states[0]);
        frontRight.setModuleState(states[1]);
        backLeft.setModuleState(states[2]);
        frontLeft.setModuleState(states[3]);
    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = -speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = -speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp; // FORWARDS
        // SmartDashboard.putNumber("Radians Chassis CMD",
        // speeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DriveConstants.KINEMATICS.toChassisSpeeds(
            backRight.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            frontLeft.getModuleState());

        return speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        states[0] = backRight.getModulePosition();
        states[1] = frontRight.getModulePosition();
        states[2] = backLeft.getModulePosition();
        states[3] = frontLeft.getModulePosition();

        return states;
    }
}
