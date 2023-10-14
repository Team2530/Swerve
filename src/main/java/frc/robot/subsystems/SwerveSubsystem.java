package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {
    SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FL_STEER_ID, SwerveModuleConstants.FL_DRIVE_ID,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FL_OFFSET_RADIANS,
            SwerveModuleConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FL_MOTOR_REVERSED);

    SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FR_STEER_ID, SwerveModuleConstants.FR_DRIVE_ID,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.FR_OFFSET_RADIANS,
            SwerveModuleConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.FR_MOTOR_REVERSED);

    SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BR_STEER_ID, SwerveModuleConstants.BR_DRIVE_ID,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BR_OFFSET_RADIANS,
            SwerveModuleConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BR_MOTOR_REVERSED);

    SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BL_STEER_ID, SwerveModuleConstants.BL_DRIVE_ID,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveModuleConstants.BL_OFFSET_RADIANS,
            SwerveModuleConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            SwerveModuleConstants.BL_MOTOR_REVERSED);


    PowerDistribution pdh = new PowerDistribution(1,ModuleType.kRev);
    int[] pdh_channels = {
        18,19,
        0,1,
        16,17,
        2,3
    };

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private Field2d field = new Field2d();

    // TODO: Properly set starting pose
    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS,
            getRotation2d(),
            getModulePositions(), new Pose2d());

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
        // double rads = getPose().getRotation().getRadians();
        odometry.update(geRotation2dOdometry(), getModulePositions());

        // TODO: Test
        // WARNING: REMOVE IF USING TAG FOLLOW!!!
        // odometry.addVisionMeasurement(LimelightHelpers.getBotPose2d(null),
        // Timer.getFPGATimestamp());

        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);

        SmartDashboard.putString("Robot Pose",
                getPose().toString());

        // SmartDashboard.putNumber("Spin Velocity", (getPose().getRotation().getRadians() - rads) / 0.02);
    
        double swerveCurrent = 0;
        for (int chan : pdh_channels)
            swerveCurrent += pdh.getCurrent(chan);
        SmartDashboard.putNumber("SwerveSubsystem Amps", swerveCurrent);
        SmartDashboard.putNumber("PDH Amps", pdh.getTotalCurrent());
    }

    public void zeroHeading() {
        navX.reset();
    }

    public void setHeading(double deg) {
        zeroHeading();
        navX.setAngleAdjustment(deg);
    }

    public Pose2d getPose() {
        Pose2d p = odometry.getEstimatedPosition();
        // - Y!!!
        p = new Pose2d(p.getX(), -p.getY(), p.getRotation().div(-1).rotateBy(new Rotation2d(Math.PI / 2.0)));
        return p;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getHeading() {
        return Units.degreesToRadians(Math.IEEEremainder(navX.getAngle(), 360));
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    public Rotation2d geRotation2dOdometry() {
        return new Rotation2d(getHeading() + Math.PI / 2);
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
        frontLeft.setModuleState(states[3]);
        frontRight.setModuleState(states[1]);
        backLeft.setModuleState(states[2]);
        backRight.setModuleState(states[0]);
    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        double tmp = -speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = -speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = tmp; // FORWARDS
        // SmartDashboard.putNumber("Radians Chassis CMD",
        // speeds.omegaRadiansPerSecond);
        speeds.omegaRadiansPerSecond *= -1;
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModules(states);
    }

    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DriveConstants.KINEMATICS.toChassisSpeeds(frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(), backRight.getModuleState());

        return speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        states[3] = frontLeft.getModulePosition();
        states[1] = frontRight.getModulePosition();
        states[2] = backLeft.getModulePosition();
        states[0] = backRight.getModulePosition();

        return states;
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulate_step();
        frontRight.simulate_step();
        backLeft.simulate_step();
        backRight.simulate_step();
    }

    public void drive(double strafeX, double strafeY, double rotate, boolean fieldOrientated) {
        ChassisSpeeds chassisSpeed;

        if (fieldOrientated) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(strafeX, strafeY, rotate,
                    getRotation2d());
        } else {
            chassisSpeed = new ChassisSpeeds(strafeX, strafeY, rotate);
        }

        setChassisSpeedsAUTO(chassisSpeed);
    }
}
