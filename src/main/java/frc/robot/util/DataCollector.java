package frc.robot.util;

import java.io.*;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.zip.GZIPOutputStream;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class contains a system for logging numerical data, mostly relating to the drivebase. 
 * 
 * Future versions may add more information to logs.
 * 
 * GZip compression is used to make the log files more compact.
 * 
 * Currently, {@link DataCollector#DataCollector(String)} is the best constructor to use, and should be used in all cases where you need to write to the disk.
 * 
 * After making ANY changes to the file format, please increment {@link DataCollector#VERSION} by 1.
 */
public class DataCollector {

    public static final byte VERSION = 1;

    /**
     * A wrapper around the passed-in OutputStream. This handles many useful byte-related functions out of the box.
     */
    private DataOutputStream data;

    /* Timestamp of the last collected frame */
    private long lastMS;

    /**
     * @param fileName A filename for the logging file. This is set in {@link Robot#disabledExit()}, and can be modified in {@link Constants.LoggingConstants}.
     */
    public DataCollector(String fileName) throws IOException {
        this(new FileOutputStream(makeTheFilenameExist(fileName)));
    }

    /* Use this constructor to specify an alternate outputstream. */
    public DataCollector(OutputStream outputStream) {
        try {
            this.data = new DataOutputStream(new GZIPOutputStream(outputStream));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Ensures that a given file path exists, and creates parent directories if needed.
     * @param fileName The filename/path to check.
     * @return The filename you passed in.
     */
    private static String makeTheFilenameExist(final String fileName) {
        Paths.get(fileName).getParent().toFile().mkdirs();
        return fileName;
    }

    /**
     * Writes the header section of the format. This currently includes a magic number (2530), the version byte and the full timestamp.
     */
    public void writeHeader() {
        try {
            data.writeShort(0x2530);
            data.writeByte(VERSION);
            this.lastMS = System.currentTimeMillis();
            data.writeUTF(Instant.now().toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Collects an individual 'frame' of information.  
     * @param robot The robot instance.
     */
    public void collectFrame(Robot robot) {
        try {
            final long thisMS = System.currentTimeMillis();
            data.writeShort((int) (thisMS - lastMS));
            data.writeByte(getRobotMode(robot));
            
            final SwerveSubsystem swerve = robot.m_robotContainer.swerveDriveSubsystem;

            //TODO: swerve.navX
            //TODO: swerve.odometry

            data.writeDouble(swerve.getHeading());

            for(int swerveModuleIndex = 0; swerveModuleIndex < 4; swerveModuleIndex ++) {
                final SwerveModule swerveModule = swerve.modules[swerveModuleIndex];

                data.writeDouble(swerveModule.driveMotor.get());

                data.writeDouble(swerveModule.getDrivePosition());
                data.writeDouble(swerveModule.getDriveVelocity());

                data.writeDouble(swerveModule.steerMotor.get());
                data.writeDouble(swerveModule.getSteerPosition());
                data.writeDouble(swerveModule.getSteerVelocity());

                data.writeDouble(swerveModule.steerPID.getP());
                data.writeDouble(swerveModule.steerPID.getI());
                data.writeDouble(swerveModule.steerPID.getD());
            }


            this.lastMS = thisMS;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public byte getRobotMode(Robot robot) {
        // 0 = disabled, 1 = auto, 2 = teleop, 3 = test
        if(robot.isAutonomous()) return 1;
        if(robot.isTeleop()) return 2;
        if(robot.isTest()) return 3;
        return 0;    
    }

    /**
     * Closes the data stream. After using this method, please get rid of the DataCollector instance as to not cause errors.
     */
    public void stop() {
        try {
            data.close();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }
}
