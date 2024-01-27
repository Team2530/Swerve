package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.util.Units;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;

    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;
    
    private final ProfiledPIDController ORIENTED_ROTATION_PID = new ProfiledPIDController(DriveConstants.ORIENTED_ROTATION_P, DriveConstants.ORIENTED_ROTATION_I, DriveConstants.ORIENTED_ROTATION_D, new Constraints(DriveConstants.ORIENTED_ROTATION_MAX_VELOCITY, DriveConstants.ORINETED_ROTATION_MAX_ACCELERATION));
    private double ORIENTATION = 0;

    private enum DriveState {
        Free,
        Locked
    };
    private enum RotationState {
        Free,
        Oriented
    };

    private DriveState driveState = DriveState.Free;
    private RotationState rotationState = RotationState.Free; 

    public DriveCommand(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.swerveSubsystem = swerveSubsystem;
        this.xbox = xbox;

        dsratelimiter.reset(SLOWMODE_MULT);

        ORIENTED_ROTATION_PID.enableContinuousInput(-180, 180);

        addRequirements(swerveSubsystem);
    }

    double clamp(double v, double mi, double ma) {
        return (v < mi) ? mi : (v > ma ? ma : v);
    }

    public Translation2d DeadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            // TODO: Check is it sqrt2 or 1.0...
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    clamp(result.getX(), -1.0, 1.0),
                    clamp(result.getY(), -1.0, 1.0));
        }
    }

    public double DeadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {  

        Translation2d xyRawLeft = new Translation2d(xbox.getLeftX(), xbox.getLeftY());
        Translation2d xySpeedLeft = DeadBand(xyRawLeft, 0.15);
        Translation2d xySpeedRight = DeadBand(new Translation2d(xbox.getRightX(), xbox.getRightY()), ControllerConstants.ORIENTED_ROTATION_DEADZONE); 

        if (Math.abs(xySpeedRight.getX()) > 0 || Math.abs(xySpeedRight.getY()) > 0){
            this.ORIENTATION = Units.radiansToDegrees(Math.atan2(xySpeedRight.getX(), -xySpeedRight.getY()));
        }

        double zSpeed;
        double xSpeed = xySpeedLeft.getX(); // xbox.getLeftX();
        double ySpeed = xySpeedLeft.getY(); // xbox.getLeftY();

        // Rotation state switch logic
        if (xbox.getRightStickButtonPressed()) {
            rotationState = RotationState.values()[RotationState.values().length > rotationState.ordinal() + 1 
                ? rotationState.ordinal() + 1 
                : 0
                ];  
        }

        // Rotation state execution logic 
        switch(rotationState){
            case Oriented:
                zSpeed = ORIENTED_ROTATION_PID.calculate(-Units.radiansToDegrees(swerveSubsystem.getHeading()), this.ORIENTATION);
                break;
            case Free: default: 
                zSpeed = DeadBand(xbox.getRightX(), 0.1); 
                break;
        } 

        // TODO: Full speed!
        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;

        // double dmult = dsratelimiter.calculate(xbox.getRightBumper() ? 1.0 :
        // SLOWMODE_MULT);
        double dmult = dsratelimiter
                .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
        xSpeed *= dmult;
        ySpeed *= dmult;
        zSpeed *= dmult;

        if (xbox.getXButton()) {
            swerveSubsystem.zeroHeading();
            swerveSubsystem.resetOdometry(new Pose2d());
            swerveSubsystem.zeroHeading();
        }

        ChassisSpeeds speeds;

        // Drive Non Field Oriented
        if (!xbox.getLeftBumper()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, -zSpeed,
                    new Rotation2d(
                            -swerveSubsystem.getRotation2d().rotateBy(DriveConstants.NAVX_ANGLE_OFFSET).getRadians()));
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, -zSpeed);
        }

        // Drive state transition logic
        switch (driveState) {
            case Free:
                driveState = xbox.getBButton() ? DriveState.Locked : DriveState.Free;
                break;
            case Locked:
                driveState = ((xyRawLeft.getNorm() > 0.15) && !xbox.getBButton()) ? DriveState.Free : DriveState.Locked;
                break;
        }

        // Drive execution logic
        switch (driveState) {
            case Free:
                SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
                swerveSubsystem.setModules(calculatedModuleStates);
                break;
            case Locked:
                swerveSubsystem.setXstance();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopDrive();
    }

    public boolean isFinished() {
        return false;
    }
}
