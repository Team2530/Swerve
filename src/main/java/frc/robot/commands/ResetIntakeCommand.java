package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class ResetIntakeCommand extends CommandBase {
    private final Intake intake;
    private final DigitalInput limitsw;

    private boolean done;

    public ResetIntakeCommand(Intake intake, DigitalInput limit) {
        this.limitsw = limit;
        this.intake = intake;

        done = false;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Limit switch is initially high (disconnected!)
        if (limitsw.get()) {
            System.out.println("ResetIntakeCommand.ResetIntakeCommand() ERROR: Limitswitch initially depressed!");
            done = true;
        }
        done = false;

        intake.enableStateControl(false);
        intake.setActuatorRaw(-0.1);

        System.out.println("Starting Intake Homing");
    }

    @Override
    public void execute() {
        if (!done) {
            // Switch is NORMALLY CLOSED
            if (limitsw.get()) {
                // Done

                intake.setActuatorRaw(0.0);
                double zeroangle = intake.getEncoderAngleRaw() - IntakeConstants.SWITCH_OFFSET_RADIANS;
                intake.setZeroAngleRaw(zeroangle);

                // intake.enableStateControl(true);
                // intake.setIntakeState(IntakeState.STOWED);

                System.out.printf("Homing done: zero set to %f radians\n", zeroangle);

                done = true;
            } else {
                intake.enableStateControl(false);
                intake.setActuatorRaw(-0.17);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.enableStateControl(true);
        done = false;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}