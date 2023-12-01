// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 2;
  }

  public static class SwerveModuleConstants {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEERING_GEAR_RATIO = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double DRIVE_GEAR_RATIO = 1.d / 6.75d;

    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_METERS_PER_SECOND = DRIVE_ROTATION_TO_METER / 60d / 60d;

    // Actual drive gains
    // public static final double MODULE_KP = 0.5;
    // public static final double MODULE_KD = 0.03;

    // NOTE: This may need additional tuning!
    public static final double MODULE_KP = 1.7; //.756 // 0.7491;  //0.56368;
    public static final double MODULE_KD = 0.0066806;// 0.0057682; //0.0076954;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 2;
    public static final int FL_STEER_ID = 1;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FL_OFFSET_ROTATIONS = -0.073730;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FL_DRIVE_MOTOR_REVERSED = true;
    public static final boolean FL_STEER_MOTOR_REVERSED = true;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 4;
    public static final int FR_STEER_ID = 3;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 4;
    public static final double FR_OFFSET_ROTATIONS = -0.935547;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FR_DRIVE_MOTOR_REVERSED = true;
    public static final boolean FR_STEER_MOTOR_REVERSED = true;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 6;
    public static final int BR_STEER_ID = 5;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 2;
    public static final double BR_OFFSET_ROTATIONS = -0.150879;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_DRIVE_MOTOR_REVERSED = true;
    public static final boolean BR_STEER_MOTOR_REVERSED = true;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 8;
    public static final int BL_STEER_ID = 7;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 3;
    public static final double BL_OFFSET_ROTATIONS = -0.888916;
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_DRIVE_MOTOR_REVERSED = true;
    public static final boolean BL_STEER_MOTOR_REVERSED = true;

  }

  public static class DriveConstants {
    public static final double MAX_MODULE_VELOCITY = 4.8;
    public static final double MAX_ROBOT_VELOCITY = 4.8;
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    // TODO: Change based on actual robot!
    public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(18.75);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0), // Back Right
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0), // Front Right
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), // Back Left
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0)); // Front Left

    public static final double XY_SPEED_LIMIT = 0.8;
    public static final double Z_SPEED_LIMIT = 1.0;
  }

  public static class IntakeConstants {
    // Intake ports
    public static final int WHEEL_LEFT_PORT = 9;
    public static final int WHEEL_RIGHT_PORT = 10;
    public static final int INTAKE_CURRENT_LIMIT = 40;

    public static final int ACTUATOR_MOTOR_PORT = 11;
    public static final int ACTUATOR_ENCODER_PORT = 5;
    // CANcoder #5 reading when intake is vertical
    public static final double OFFSET_RADIANS = Units.degreesToRadians(102.0);
    public static final double ACTUATOR_GEAR_RATIO = (30.f/28.f);

    public static final double MAX_SPEED = 0.50;
  }
}
