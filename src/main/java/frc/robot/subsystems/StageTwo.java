package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class StageTwo extends ProfiledPIDSubsystem {

    private final TalonFX stageTwoMotor = new TalonFX(ArmConstants.STAGE_TWO_MOTOR_PORT);

    private final CANcoder stageTwoEncoder = new CANcoder(ArmConstants.STAGE_TWO_ENCODER_PORT);

    private double stageOneOffset = 0.0;


    public StageTwo() {
        super(ArmConstants.STAGE_TWO_PROFILEDPID);

        stageTwoMotor.setInverted(ArmConstants.STAGE_TWO_ISREVERSED);
        stageTwoMotor.setSafetyEnabled(false);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // setpoint doesn't really matter as we aren't using a feedforward as of right now
        stageTwoMotor.setVoltage(output);
    }

    @Override
    protected double getMeasurement() {
         return Units.rotationsToRadians((stageTwoEncoder.getPosition().getValueAsDouble()
                * (ArmConstants.STAGE_TWO_ENCODER_ISREVERSED ? -1 : 1))
                + ArmConstants.STAGE_TWO_ENCODER_OFFSET + stageOneOffset);
    }

    public void updateStageOneOffset(double offset) {
        stageOneOffset = offset;
    }

    public void setGoalDegrees(double degrees) {
        setGoal(Units.degreesToRadians(degrees));
    }


    
}
