package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private WPI_TalonSRX actuatorMotor = new WPI_TalonSRX(IntakeConstants.ACTUATOR_MOTOR_PORT);
    private CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.WHEEL_LEFT_PORT, MotorType.kBrushless);
    private CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.WHEEL_RIGHT_PORT, MotorType.kBrushless);

    private CANCoder actuatorEncoder = new CANCoder(IntakeConstants.ACTUATOR_ENCODER_PORT);
    private final double actuatorOffsetRadians;

    private final PIDController actuatorPID = new PIDController(0.00001, 0.0, 0.0);

    public static enum IntakeState {
        STOWED(90),
        PICKUP(-30),
        PLACE(0),
        HIGH(50),
        LOW(20);

        private double angleDegrees;

        IntakeState(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }

    private IntakeState intakeState = IntakeState.STOWED;

    private final XboxController xbox;

    public Intake(double actuatorOffsetRadians, XboxController xbox) {
        this.actuatorOffsetRadians = actuatorOffsetRadians;
        this.xbox = xbox;
    }

    @Override
    public void periodic() {
        double currentAngle = actuatorEncoder.getAbsolutePosition() * (Math.PI / 180d);

        actuatorMotor.set(actuatorPID.calculate(currentAngle, Units.degreesToRadians(intakeState.angleDegrees)));

        // Set intake state based on the xbox POV
        switch (xbox.getPOV()) {
            case -1:
                break;
            case 0:
                setIntakeState(IntakeState.STOWED);
                break;
            case 315:
                setIntakeState(IntakeState.HIGH);
                break;
            case 180:
                setIntakeState(IntakeState.PICKUP);
                break;
            case 270:
                setIntakeState(IntakeState.PLACE);
                break;
        }

    }

    public void set(double speed) {
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
    }
}
