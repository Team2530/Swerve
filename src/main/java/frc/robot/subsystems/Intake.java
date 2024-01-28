package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.IntakeCommand;

public class Intake extends SubsystemBase {

    public enum IntakeMode {
         IDLE(0.2),
         STOPED(0.0),
         INTAKING(0.3),
         REVERSE(-0.2),
         CUSTOM(1.5);

         private double modeSpeed;

         private IntakeMode(double modeSpeed) {
            this.modeSpeed = modeSpeed;
         }
    }

    // Current Wanted setpoint of intake
    private IntakeMode intakeMode = IntakeMode.STOPED;

    // Falcon 500 intake motor
    private final TalonFX intakeMotor = new TalonFX(ArmConstants.INTAKE_MOTOR_PORT);

    // desired custom motor output percent
    private double outputPercent = 0.0;

    private final PIDController intakeProfile = new PIDController(0.01, 0.0, 0.0);

    public Intake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        double percent = intakeProfile.calculate(intakeMotor.get(), outputPercent);
        intakeMotor.set(percent);
    }

    /**
     * Sets intake mode
     * @param mode {@link IntakeMode} desired intake mode
     */
    public void setIntakeMode(IntakeMode mode) {
        intakeMode = mode;
        outputPercent = intakeMode.modeSpeed;
    }

    /**
     * Changes the intake state to custom, and sets motor to custom output
     * @param percent output percent
     */
    public void setCustomPercent(double percent) {
        intakeMode = IntakeMode.CUSTOM;
        outputPercent = percent;
    }

    public double getOutputPercent() {
        return outputPercent;
    }

    public void coast() {
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void brake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean getFrontLimitClosed() {
        return intakeMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getReverseLimitClosed() {
        return intakeMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }
}
