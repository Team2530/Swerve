package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {

    public enum IntakeMode {
         IDLE(0.2),
         STOPPED(0.0),
         INTAKING(0.2),
         REVERSE(-0.1),
         CUSTOM(1.5);

         private double modeSpeed;

         private IntakeMode(double modeSpeed) {
            this.modeSpeed = modeSpeed;
         }
    }

    // Current Wanted setpoint of intake
    private IntakeMode intakeMode = IntakeMode.STOPPED;

    // Falcon 500 intake motor
    private final TalonFX intakeMotor = new TalonFX(ArmConstants.INTAKE_MOTOR_PORT);

    // desired custom motor output percent
    private double outputPercent = 0.0;

    // private final PIDController intakeProfile = new PIDController(0.1, 0.0, 0.01);

    // allow motor to speed up quickly and slow down over a period of time
    private final SlewRateLimiter intakeProfile = new SlewRateLimiter(5, -10, 0.0);

    public Intake() {
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        double percent = intakeProfile.calculate(outputPercent);
        intakeMotor.set(percent);

        SmartDashboard.putNumber("Intake Percent", percent * 100);
        SmartDashboard.putBoolean("Intake FWD Limit", getFrontLimitClosed());
        SmartDashboard.putBoolean("Intake REV Limit", getReverseLimitClosed());

    }

    /**
     * Sets intake mode
     * @param mode {@link IntakeMode} desired intake mode
     */
    public void setMode(IntakeMode mode) {
        intakeMode = mode;
        outputPercent = intakeMode.modeSpeed;

        SmartDashboard.putString("Shootake", "Intake mode set to " + (intakeMode.name()));
    }

    /**
     * Changes the intake state to custom, and sets motor to custom output
     * @param percent output percent from (-1 to 1)
     */
    public void setCustomPercent(double percent) {
        intakeMode = IntakeMode.CUSTOM;
        // clamp between (-1,1)
        outputPercent = Math.max(-1, Math.min(percent, 1));

        SmartDashboard.putString("Shootake", "Intake speed set to " + String.format("%.0f", percent * 100) + " percent");
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
