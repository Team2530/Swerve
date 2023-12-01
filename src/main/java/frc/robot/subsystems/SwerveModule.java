package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;;

public class SwerveModule {
    private static int moduleNumber = 0;
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private final CANcoder absoluteEncoder;

    private final SparkMaxPIDController steerPID;

    private final int thisModuleNumber;

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double encoderOffsetRotations,
            boolean isAbsoluteEncoderReversed, boolean driveMotorInverted, boolean steerMotorInverted) {
        driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);

        steerMotor = new CANSparkMax(steerCanID, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setInverted(steerMotorInverted);

        driveMotorEncoder = driveMotor.getEncoder();
        steerMotorEncoder = steerMotor.getEncoder();

        absoluteEncoder = new CANcoder(absoluteEncoderPort);
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        // Set the CANCoder magnetic offset. This is the inverse of the ROTATIONS the sensor reads when the wheel is pointed straight forward.
        canCoderConfig.MagnetSensor.MagnetOffset = encoderOffsetRotations;
        // Set CANCoder to return direction from [-0.5, 0.5) - straight forward should be 0
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // Set the CANCoder phase / direction
        canCoderConfig.MagnetSensor.SensorDirection =
            isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        // Drive distance in meters
        driveMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER);
        // Drive velocity in meters per second
        driveMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_SECOND);
        driveMotor.burnFlash();

        // Steer position in rotations
        steerMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.STEERING_GEAR_RATIO);

        steerPID = steerMotor.getPIDController();
        steerPID.setP(SwerveModuleConstants.MODULE_KP);
        steerPID.setD(SwerveModuleConstants.MODULE_KD);
        steerPID.setPositionPIDWrappingEnabled(true);
        steerPID.setPositionPIDWrappingMaxInput(0.5);
        steerPID.setPositionPIDWrappingMinInput(-0.5);
        steerMotor.burnFlash();

        thisModuleNumber = moduleNumber;
        moduleNumber++;

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotorEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity();
    }

    /**
     * Get steer position
     * @return steer position in range [-Pi, Pi) radians
     */
    public Rotation2d getSteerPosition() {
        var rotations = steerMotorEncoder.getPosition();
        return Rotation2d.fromRotations(rotations);
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    /**
     * Gets the absolute encoder (CANCoder) position
     * @return position in rotations [-0.5, 0.5)
     */
    private double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition().waitForUpdate(0.4).getValue();
    }

    private void resetEncoders() {
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getSteerPosition());
        double drive_command = state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY;
        driveMotor.set(drive_command);
        steerPID.setReference(state.angle.getRotations(), ControlType.kPosition);

        SmartDashboard.putNumber("Drive" + thisModuleNumber, drive_command);
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
