package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private final CANCoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;

    private final PIDController steerPID;

    private static int moduleNumber = 0;

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed) {
        driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerCanID, MotorType.kBrushless);

        driveMotorEncoder = driveMotor.getEncoder();
        steerMotorEncoder = steerMotor.getEncoder();

        absoluteEncoder = new CANCoder(absoluteEncoderPort);

        this.motorOffsetRadians = motorOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        driveMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER);
        driveMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_MINUTE);

        steerMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.STEER_ROTATION_TO_RADIANS);
        steerMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.STEER_RADIANS_PER_MINUTE);

        steerPID = new PIDController(SwerveModuleConstants.MODULE_KP, 0, SwerveModuleConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        moduleNumber++;

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotorEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity();
    }

    public double getSteerPosition() {
        return steerMotorEncoder.getPosition();
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = absoluteEncoder.getAbsolutePosition() * Math.PI / 180;
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition() * Math.PI / 180);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, new Rotation2d(getSteerPosition()));
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY * 0.5);
        steerMotor.set(steerPID.calculate(getSteerPosition(), state.angle.getRadians()));

    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
