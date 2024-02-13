package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import static java.lang.Math.*;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
    // -------- First Joint ------- \\
    private final TalonFX stageOneLeader = new TalonFX(ArmConstants.STAGE_ONE_MOTOR_L);
    private final TalonFX stageOneFollower = new TalonFX(ArmConstants.STAGE_ONE_MOTOR_R);
    
    private final CANcoder stageOneEncoder = new CANcoder(ArmConstants.STAGE_ONE_ENCODER_PORT);

    private final ArmFeedforward stageOneFeedForward = ArmConstants.STAGE_ONE_FEEDFORWARD;
    private final ProfiledPIDController stageOneFeedback = ArmConstants.STAGE_ONE_PROFILEDPID;

    private double shoulderSetpoint = 0;
    private double shoulderVelocity = 0;

    // -------- Second Joint ------ \\
    private final TalonFX stageTwoMotor = new TalonFX(ArmConstants.STAGE_TWO_MOTOR_PORT);

    private final CANcoder stageTwoEncoder = new CANcoder(ArmConstants.STAGE_TWO_ENCODER_PORT);

    private final ArmFeedforward stageTwoFeedforward = ArmConstants.STAGE_TWO_FEEDFORWARD;
    private final ProfiledPIDController wristFeedback = ArmConstants.STAGE_TWO_PROFILEDPID;

    private double wristSetpoint = 0;
    private double wristVelocity = 0;

    // -------- SysId -------------- \\
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private SysIdRoutine SHOULDER_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((voltage) -> {stageOneLeader.setVoltage(voltage.magnitude());}, this::sysIdShoulderMotorLog, this)
    );
    private SysIdRoutine WRIST_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((voltage) -> {stageTwoMotor.setVoltage(voltage.magnitude());}, this::sysIdWristMotorLog, this)
    );

    // -------- Smartdashboard ----- \\
    Mechanism2d mech = new Mechanism2d(2, 2);
    MechanismRoot2d arm = mech.getRoot("arm", 0, 0);
    MechanismLigament2d shoulder = arm.append(new MechanismLigament2d("shoulder", 3, 90));
    MechanismLigament2d wrist = shoulder.append(new MechanismLigament2d("wrist", 1, 0));

    // -------- Other -------------- \\ 
    // private final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
    
    public static enum ControlState {
        Human,
        Homing
    }
    private ControlState controlState = ControlState.Human;

        
    // ---------------------------- \\

    private final VoltageOut request = new VoltageOut(0);


    public Arm() {
        // persistant variables
        Preferences.initDouble(ArmConstants.STAGE_ONE_OFFSET_KEY, ArmConstants.STAGE_ONE_ENCODER_OFFSET);
        Preferences.initDouble(ArmConstants.STAGE_TWO_OFFSET_KEY, ArmConstants.STAGE_TWO_ENCODER_OFFSET);

        // motos n things        
        stageOneLeader.setInverted(ArmConstants.L_STAGE_ONE_ISREVERSED);
        stageOneFollower.setControl(new Follower(stageOneLeader.getDeviceID(), ArmConstants.FOLLOWER_STAGE_ONE_ISREVERSED));

        stageTwoMotor.setInverted(ArmConstants.STAGE_TWO_ISREVERSED);

        stageOneLeader.setSafetyEnabled(false);        
        stageOneFollower.setSafetyEnabled(false);
        stageTwoMotor.setSafetyEnabled(false);

    }

    @Override
    public void periodic() {
        shoulder.setAngle(stageOneEncoder.getPosition().getValue());
        wrist.setAngle(stageTwoEncoder.getPosition().getValue());
        SmartDashboard.putData("Arm", mech); 

        switch (controlState) {
            case Human:
                // find target
                // lerp towards target
                // yippee.mp4
                break;
            case Homing:
                // if (limitSwitch.get()) {
                //     Preferences.setDouble(ArmConstants.STAGE_ONE_OFFSET_KEY, stageOneEncoder.getPosition().getValue());
                //     controlState = ControlState.Human;
                // }
                break;
        }
        // shoulder


        stageOneLeader.set(Math.max(0, Math.min( 0.4 ,ArmConstants.STAGE_ONE_PROFILEDPID.calculate(stageOneEncoder.getPosition().getValue(), shoulderSetpoint))));

        SmartDashboard.putNumber("L_POS", stageOneLeader.getPosition().getValue());
        SmartDashboard.putNumber("actual", stageOneLeader.getMotorVoltage().getValue());
        SmartDashboard.putNumber("voltage", ArmConstants.STAGE_ONE_PROFILEDPID.calculate(stageOneEncoder.getPosition().getValue(), shoulderSetpoint + Preferences.getDouble(ArmConstants.STAGE_ONE_OFFSET_KEY, ArmConstants.STAGE_ONE_ENCODER_OFFSET)) +
            ArmConstants.STAGE_ONE_FEEDFORWARD.calculate(shoulderSetpoint, shoulderVelocity));
        // wrist
        stageTwoMotor.setVoltage(
            wristFeedback.calculate(stageTwoEncoder.getPosition().getValue(), wristSetpoint + Preferences.getDouble(ArmConstants.STAGE_TWO_OFFSET_KEY, ArmConstants.STAGE_TWO_ENCODER_OFFSET)) +
            ArmConstants.STAGE_TWO_FEEDFORWARD.calculate(wristSetpoint, wristVelocity)
        );

        SmartDashboard.putBoolean("left fwd limit", stageOneFollower.getReverseLimit().getValue().value == 1);
        SmartDashboard.putBoolean("right fwd limit", stageOneLeader.getReverseLimit().getValue().value == 1);

    }



    public void setWrist(double setpoint) {
        if (controlState == ControlState.Human) {
            wristSetpoint = setpoint;
        }
    }
    public void setWristVelocity(double velocity) {
        if (controlState == ControlState.Human){
            wristVelocity = velocity;
        } 
    }

    public void setShoulder(double setpoint) {
        if (controlState == ControlState.Human){
            shoulderSetpoint = setpoint;
        } 
    }
    public void setShoulderVelocity(double velocity) {
        if (controlState == ControlState.Human){
            shoulderVelocity = velocity;
        }
    }

    public Command homeArmCommand() {
        return new Command () {
            @Override
            public void initialize() {
                controlState = ControlState.Homing;
                shoulderSetpoint = 0;
                wristSetpoint = 0;                      
            } 
        };
    }

    

    public Translation2d getFirstLinkEndpoint() {
        final double angle1 = stageOneEncoder.getPosition().getValueAsDouble();

        return new Translation2d(sin(angle1), -cos(angle1)).times(ArmConstants.STAGE_ONE_LENGTH);
    }

    public Translation2d getCurrentEndpoint() {
        final double angle1 = stageOneEncoder.getPosition().getValueAsDouble();
        final double angle2 = stageTwoEncoder.getPosition().getValueAsDouble();

        final Translation2d a = new Translation2d(sin(angle1), -cos(angle1)).times(ArmConstants.STAGE_ONE_LENGTH);
        final Translation2d b = new Translation2d(sin(angle1 + angle2), -cos(angle1 + angle2)).times(ArmConstants.STAGE_TWO_LENGTH);

        return a.plus(b);
    }
    
    public static double secondJointIK(double x, double y) {
        return acos((x * x + y * y - ArmConstants.STAGE_ONE_LENGTH * ArmConstants.STAGE_ONE_LENGTH - ArmConstants.STAGE_TWO_LENGTH * ArmConstants.STAGE_TWO_LENGTH) / (2 * ArmConstants.STAGE_ONE_LENGTH * ArmConstants.STAGE_TWO_LENGTH));
    }

    public static double firstJointIK(double x, double y, double z2) {
        return atan2(y, x) - atan2(ArmConstants.STAGE_TWO_LENGTH * sin(z2), ArmConstants.STAGE_ONE_LENGTH + ArmConstants.STAGE_TWO_LENGTH * cos(z2));
    }

    public static double[] armInverseKinematics(double x, double y) {
        double z2 = secondJointIK(x, y);
        double z1 = firstJointIK(x, y, z2);
        return new double[] {z1, z2};
    }



    public void sysIdShoulderMotorLog(SysIdRoutineLog log){
         log.motor("shoulder-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            stageOneLeader.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(stageOneEncoder.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(stageOneLeader.getVelocity().getValue(), MetersPerSecond));
        log.motor("shoulder-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            stageOneFollower.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(stageOneFollower.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(stageOneFollower.getVelocity().getValue(), MetersPerSecond));
    }

    public void sysIdWristMotorLog(SysIdRoutineLog log){
         log.motor("wrist")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            stageTwoMotor.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(stageTwoMotor.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(stageTwoMotor.getVelocity().getValue(), MetersPerSecond));
    }

    public Command shoulderQuasiCommand(Direction direction){
        return SHOULDER_sysIdRoutine.quasistatic(direction);
    }

    public Command shoulderDynamicCommand(Direction direction){
        return SHOULDER_sysIdRoutine.dynamic(direction);
    }

    public Command wristQuasiCommand(Direction direction){
        return WRIST_sysIdRoutine.quasistatic(direction);
    }
    
    public Command wristDynamicCommand(Direction direction){
        return WRIST_sysIdRoutine.dynamic(direction);
    }
}