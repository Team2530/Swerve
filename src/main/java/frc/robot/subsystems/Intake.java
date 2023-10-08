package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private WPI_VictorSPX actuatorMotor = new WPI_VictorSPX(IntakeConstants.ACTUATOR_MOTOR_PORT);
    private CANSparkMax leftIntake;
    private CANSparkMax rightIntake;

    private CANCoder actuatorEncoder = new CANCoder(IntakeConstants.ACTUATOR_ENCODER_PORT);

    private final PIDController actuatorPID = new PIDController(1.25, 0.0, 0.0);

    private double ANGLE_OFFSET = IntakeConstants.OFFSET_RADIANS;

    public static enum IntakeState {
        STOWED(-30),
        // STOWED(-),
        PICKUP(120),
        LOW(70),
        HIGH(45),
        TIPPEDCONE_CUBE(110);

        private double angleDegrees;

        IntakeState(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }

    private IntakeState intakeState = IntakeState.STOWED;
    private boolean statectl_enabled = true;

    private final XboxController xbox;

    private double stateDegrees;

    public Intake(XboxController xbox) {
        this.xbox = xbox;

        this.leftIntake = new CANSparkMax(IntakeConstants.WHEEL_LEFT_PORT, MotorType.kBrushless);
        this.rightIntake = new CANSparkMax(IntakeConstants.WHEEL_RIGHT_PORT, MotorType.kBrushless);

        leftIntake.setIdleMode(IdleMode.kBrake);
        rightIntake.setIdleMode(IdleMode.kBrake);

        enableCurrentControl(true);
        enableStateControl(true);

        Preferences.initDouble("ANGLE_OFFSET", IntakeConstants.OFFSET_RADIANS);
        ANGLE_OFFSET = Preferences.getDouble("ANGLE_OFFSET", IntakeConstants.OFFSET_RADIANS);
        System.out.printf("Angle offset is %f\n", ANGLE_OFFSET);
    }

    public void enableCurrentControl(boolean currentcontrol) {
        if (currentcontrol) {
            leftIntake.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
            rightIntake.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        } else {
            leftIntake.setSmartCurrentLimit(80);
            rightIntake.setSmartCurrentLimit(80);
        }
    }

    public IntakeState getIntakeState() {
        return this.intakeState;
    }

    public void addIntakeState(double degreesmove) {
        this.stateDegrees = Math.min(Math.max(stateDegrees + degreesmove, IntakeState.STOWED.angleDegrees),
                IntakeState.PICKUP.angleDegrees);
    }

    @Override
    public void periodic() {
        double currentAngle = (Units.degreesToRadians(actuatorEncoder.getPosition())
                - ANGLE_OFFSET) * IntakeConstants.ACTUATOR_GEAR_RATIO;

        SmartDashboard.putNumber("Intake current angle", Units.degreesToRadians(actuatorEncoder.getPosition()));
        SmartDashboard.putNumber("Intake zeroed angle", currentAngle);
        if (this.statectl_enabled) {
            actuatorMotor
                    .set(Util.cap(actuatorPID.calculate(currentAngle, Units.degreesToRadians(stateDegrees)),
                            IntakeConstants.MAX_SPEED));
        }

        SmartDashboard.putNumber("Intake Current (A)", leftIntake.getOutputCurrent());
    }

    public void setActuatorRaw(double speed) {
        this.actuatorMotor.set(speed);
    }

    public void enableStateControl(boolean enabled) {
        this.statectl_enabled = enabled;
    }

    // Returns Radians
    public double getEncoderAngleRaw() {
        return Units.degreesToRadians(actuatorEncoder.getPosition());
    }

    public void setZeroAngleRaw(double radians_at_zero) {
        this.ANGLE_OFFSET = radians_at_zero;
        Preferences.setDouble("ANGLE_OFFSET", this.ANGLE_OFFSET);
    }

    public void setIntakeSpeed(double speed) {
        leftIntake.set(speed);
        rightIntake.set(-speed);
    }

    public void stop() {
        leftIntake.set(0);
        rightIntake.set(0);
        actuatorMotor.set(0);
    }

    public void setIntakeState(IntakeState state) {
        this.intakeState = state;
        this.stateDegrees = state.angleDegrees;
    }
}
