package frc.systems.arcadedrive.toplevel.config;

import edu.wpi.first.wpilibj.GenericHID;
import frc.util.PID_Config;

public final class ArcadeConfig{
    /**
     * CAN IDs for Drive Motor Controllers and Sensors
     */
    public static final int LEFT_MASTER_ID = 31;
    public static final int LEFT_SLAVE_ID = 32;

    public static final int RIGHT_MASTER_ID = 33;
    public static final int RIGHT_SLAVE_ID = 34;

    /**
     * PID Values for arcade drive
     */
    public static final PID_Config DRIVE_PID = new PID_Config(0.3, 0.002, 30.0, 0.3673, 300);

    /**
     * Ramp Rates
     */
    public static final double OPEN_LOOP_RAMP_RATE = 0.3;
    public static final double CLOSED_LOOP_RAMP_RATE = 0;

    /**
     * Max Drivetrain Speed
     */
    public static final double MAX_SPEED= 1.0;

    /**
     * Controller Inputs
     */
    public static final GenericHID.Hand STEER_AXIS = GenericHID.Hand.kLeft ; 
    public static final GenericHID.Hand THROTTLE_AXIS = GenericHID.Hand.kRight ; 
    
    public static final int REVERSE_BUTTON = 5;
}
