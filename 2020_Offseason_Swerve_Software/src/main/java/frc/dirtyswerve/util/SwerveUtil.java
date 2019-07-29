package frc.dirtyswerve.util;

import frc.util.PID_Config;

public final class SwerveUtil{
    /**
     * Length and Width of the Chassis (any unit)
     * Should be measured from where wheels make contact with the ground
     */

    public static final double CHASSIS_WIDTH = 28;
    public static final double CHASSIS_LENGTH = 28;

    /**
     * Linear Speed Factors
     */
    public static final double DISPLACEMENT_FACTOR = 1.0;
    public static final double ROTATION_FACTOR = 0.1;

    /**
     * Selected Configuration Control Modes
     */
    public static final DriveMode SELECTED_DRIVE_MODE = DriveMode.FIELD_ORIENTED;
    public static final ThrottleType SELECTED_THROTTLE_TYPE = ThrottleType.VelocityMode;

    /**
     * CAN IDs for Swerve Motor Controllers and Sensors
     */

    public static final int GYRO_ID = 11;

    public static final int FRONT_RIGHT_THROTTLE = 41;
    public static final int FRONT_RIGHT_AZIMUTH = 31;

    public static final int FRONT_LEFT_THROTTLE = 42;
    public static final int FRONT_LEFT_AZIMUTH = 32;

    public static final int REAR_RIGHT_THROTTLE = 43;
    public static final int REAR_RIGHT_AZIMUTH = 33;

    public static final int REAR_LEFT_THROTTLE = 44;
    public static final int REAR_LEFT_AZIMUTH = 34;

    /**
     * PID Values for swerve
     */
    public static final PID_Config AZIMUTH_PID = new PID_Config(0, 0, 0, 0, 0);
    public static final PID_Config THROTTLE_PID = new PID_Config(0, 0, 0, 0, 0);

    /**
     * Gear Ratios (After the encoder)
     */

    public static final int AZIMUTH_TICKS_PER_DEGREE = 10;
    public static final int THOTTLE_TICKS_PER_INCH = 50;

    /**
     * Methods of Robot Control
     */
    public enum DriveMode{
        FIELD_ORIENTED,
        ROBOT_ORIENTED
    }

    /**
     * Methods of Throttle Control
     */
    public enum ThrottleType{
        VelocityMode,
        OutputMode
    }

    /**
     * Minimum Throttle Output (Porportion of Joystick Control)
     */
    public static final double MIN_THROTTLE = 0.05;
}