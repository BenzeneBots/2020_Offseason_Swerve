package frc.systems.dirtyswerve.lowerlevel.module;

import com.revrobotics.ControlType;

import frc.systems.dirtyswerve.toplevel.config.SwerveConfig;
import frc.systems.dirtyswerve.toplevel.config.SwerveConfig.ThrottleType;;

public class SwerveModule extends SwerveModuleLowerLevel{

    /**
     * Creates a new swerve module with it's two motor controllers. Controls both azimuth (rotation) and steer
     * of a module. 
     * 
     * Instantiates each motor controller, assigning PID Loops, and configures sensor settings. Also accesses a 
     * file on the file system to get the absolute encoder offset from. If no such file exists, assumes the current 
     * position is straight and creates a new file with the allocated offset.
     * 
     * @param ModuleName Assign a Name to Each Module - This determines the File Name on the File System
     * @param throttleID Assosciated CAN ID for the Motor Controller that controls the throttle
     * @param azimuthID Assosciated CAN ID for the Motor Controller that controls the azimuth
     */
    public SwerveModule(String ModuleName,int throttleID, int azimuthID){
        super(ModuleName,throttleID,azimuthID);
    }

    /**
     * Sets the velocity of the throttle motor in inches/second
     * @param velocity Desired velocity in inches/second
     */
    public void setThrottleVelocity(double velocity){
        if(invertThrottle){
            velocity*=-1;
        }
       throttleMotor.setPIDReference(velocity/SwerveConfig.THOTTLE_TICKS_PER_INCH,ControlType.kVelocity);
    }

    /**
     * Sets the output of the throttle motor in %
     * @param output Desired output in %
     */
    public void setThrottleOutput(double output){
        if(invertThrottle){
            output*=-1;
        }
        throttleMotor.set(output);
    }

    /**
     * Processes a target azimuth to determine the shortest possible path to that angle. Accounts for the fact
     * that a 180 degree offset is the same as long as throttle velocity is inverted. If the target Azimuth is out
     * of the -180-180 range it is scaled to fit so don't worry about gyro and other offsets.
     * 
     * @param targetAzimuth Any angle at which the azimuth of the motor should be set to in degrees
     */
    public void setAzimuth(double targetAzimuth){
        targetAzimuth = Math.IEEEremainder(targetAzimuth,360);
        
        double finalAzimuth = determinePreferredAzimuth(targetAzimuth);

        setRawAzimuth(finalAzimuth+getDegreeOverrun());
    }

    /**
     * Sets all aspects of the module including the azimuth and throttle with respect to the selected mode.
     * @param azimuth desired angle for the module in degrees
     * @param throttle any value of the throttle (-1.0 to 1.0 for open-loop output mode / any value for closed-loop velocity mode)
     */
    public void set(double azimuth,double throttle){
        setAzimuth(azimuth);

        if(SwerveConfig.SELECTED_THROTTLE_TYPE == ThrottleType.VelocityMode){
            setThrottleVelocity(throttle);
        }else{
            setThrottleOutput(throttle);
        }
    }
}