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
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;;

public class SwerveModule {
    private static int moduleNumber = 0;
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private  double driveEncSim = 0;
    private  double steerEncSim = 0;

    private final CANCoder absoluteEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;
    private final boolean motor_inv;

    private final PIDController steerPID;

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

    public void simulate_step() {
        driveEncSim += 0.02 * driveMotor.get() * (DriveConstants.MAX_MODULE_VELOCITY);
        steerEncSim += 0.02 * steerMotor.get() * (10.0);
    }

    public double getDrivePosition() {
        if (Robot.isSimulation()) return driveEncSim;
        return driveMotorEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity();
    }

    public double getSteerPosition() {
        if (Robot.isSimulation()) return steerEncSim;
        return steerMotorEncoder.getPosition();
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

        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(-getSteerPosition()).rotateBy(DriveConstants.NAVX_ANGLE_OFFSET.times(-1)));
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getSteerPosition());
        double drive_command = state.speedMetersPerSecond / DriveConstants.MAX_MODULE_VELOCITY;

        driveMotor.set(drive_command * (motor_inv ? -1.0 : 1.0));

        // This is stupid
        // steerPID.setP(Constants.SwerveModuleConstants.MODULE_KP *
        // Math.abs(drive_command));
        double steercmd = steerPID.calculate(getSteerPosition(), state.angle.getRadians());
        if (Robot.isSimulation()) {
            steerMotor.set(steercmd);
        } else {
            steerMotor.setVoltage(12*steercmd);
        }
        // SmartDashboard.putNumber("Abs" + thisModuleNumber,
        // getAbsoluteEncoderPosition());
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
