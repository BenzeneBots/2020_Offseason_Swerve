package frc.systems.dirtyswerve.lowerlevel.module;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.systems.dirtyswerve.toplevel.config.SwerveConfig;
import frc.util.FileUtil;
import java.io.File;
import frc.util.BbSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class SwerveModuleLowerLevel{

    public BbSparkMax throttleMotor;

    public WPI_TalonSRX azimuthMotor;

    public double azimuthOffset; //Offset in Degrees
    public File offsetFile;

    public boolean invertThrottle;

    /**
     * Creates a new swerve module with it's two motor controllers. Controls both azimuth (rotation) and steer
     * of a module. 
     * 
     * Instantiates each motor controller, assigning PID Loops, and configures sensor settings.
     * 
     * @param ModuleName Assign a Name to Each Module - This determines the File Name on the File System
     * @param throttleID Assosciated CAN ID for the Motor Controller that controls the throttle
     * @param azimuthID Assosciated CAN ID for the Motor Controller that controls the azimuth
     */
    public SwerveModuleLowerLevel(String ModuleName,int throttleID, int azimuthID){
        throttleMotor = new BbSparkMax(throttleID, MotorType.kBrushless);
        throttleMotor.setEncPosition(0);
        SwerveConfig.THROTTLE_PID.assign(throttleMotor);
        throttleMotor.setOpenLoopRampRate(1.0);
        
        azimuthMotor = new WPI_TalonSRX(azimuthID);
        SwerveConfig.AZIMUTH_PID.assign(0, azimuthMotor);
        azimuthMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        azimuthMotor.configFeedbackNotContinuous(false,0);
        azimuthMotor.setNeutralMode(NeutralMode.Brake);
        azimuthMotor.configNeutralDeadband(0.04);   
    }

    /**
     * Creates a File Stream according to the name of the module and if a file by that name exists reads the value, if 
     * not it creates the file and the absolute azimuth is assumed to be 0. 
     * @param ModuleName The name of the swerve module
     */
    public void readOffset(String ModuleName){
        offsetFile = new File("/home/lvuser/"+ModuleName+"_AzimuthOffset.txt");
        if(offsetFile.isFile()){
            azimuthOffset = (double)FileUtil.readObjectFromFile(azimuthOffset,offsetFile);
        }else{
            resetOffset();
        }
    }

    /**
     * Gets the current position of the swerve module in degrees and writes this number to a file as the offset
     */
    public void resetOffset(){
        azimuthOffset = -azimuthMotor.getSelectedSensorPosition()/SwerveConfig.AZIMUTH_TICKS_PER_DEGREE;
        FileUtil.writeObjectToFile(offsetFile, azimuthOffset);
    }

    /**
     * Returns the sensor value in degrees
     * @return Sensor value in degrees
     */
    public double getRawAzimuth(){
        return azimuthMotor.getSelectedSensorPosition()/SwerveConfig.AZIMUTH_TICKS_PER_DEGREE;
    }

    /**
     * Polls the azimuth motor controller for the position in degrees with the offset applied
     * @return azimuth in degrees
     */
    public double getRelativeAzimuth(){
        // The Position of the Azimuth in Degrees accounting for extra rotations
        return getRawAzimuth() + azimuthOffset; 
    }

    /**
     * Gets how many degrees over/under the -180 to 180 degree range that the azimuth is.
     * 
     * @param azimuth azimuth in degrees that is outside of the 0 to 360 range
     * @return Returns multiple of 360 degrees that signifies how many rotations the azimuth has overrun
     */
    public double getDegreeOverrun(){
        return getRelativeAzimuth()-getAbsoluteAzimuth();
    }

    /**
     * Converts an azimuth outside of the -180 to 180 range into an azimuth in the -180 to 180 range
     * 
     * @param azimuth Any azimuth
     * @return Equivalent azimuth in a -180 to 180 range
     */
    public double getAbsoluteAzimuth(){
        //returns within one rotation, the azimuth of the wheel
        return Math.IEEEremainder(getRelativeAzimuth(), 360);
    }

    /**
     * Converts a degree setpoint for azimuth into raw units accounting for rotation overrun and azimuth offset
     * 
     * @param azimuth target azimuth in degrees
     */
    public void setRawAzimuth(double azimuth){
        azimuthMotor.set(ControlMode.Position,(azimuth+azimuthOffset)*SwerveConfig.AZIMUTH_TICKS_PER_DEGREE);
    }

    /**
     * Distinguishes between the two possible azimuths in a rotation that are 180 degree offsets of each other
     * and determines which azimuth is closer to the current position. Also inverts the direction of the throttle
     * if the secondary azimuth is chosen. Returns the value of the closer azimuth.
     * 
     * @param targetAzimuth The target azimuth from 0 to 360 degrees
     * @param currentAzimuth The current azimuth from 0 to 360 degrees
     * @return Returns an equivalent azimuth that is the closest to the current azimuth
     */
    public double determinePreferredAzimuth(double targetAzimuth){
        double primaryAzimuth = targetAzimuth;
        double secondaryAzimuth = getSecondaryAzimuth(primaryAzimuth);
        double currentAzimuth = this.getAbsoluteAzimuth();
        
        if(Math.abs(primaryAzimuth - currentAzimuth)<Math.abs(secondaryAzimuth-currentAzimuth)){
            invertThrottle = false;
            return primaryAzimuth;
        }else{
            invertThrottle = true;
            return secondaryAzimuth;
        }
    }

    /**
     * Creates a secondary azimuth possibility that is a 180 degree offset from the primary based on if the current
     * azimuth is less than or greater than the primary azimuth.
     * 
     * Note: Secondary Azimuth may exceed -180 to 180 range.
     * 
     * @param primaryAzimuth The original target azimuth from 0 to 360 degrees
     * @param currentAzimuth The current azimuth from 0 to 360 degrees
     * @return
     */
    public double getSecondaryAzimuth(double primaryAzimuth){
        double secondaryAzimuth;
        double currentAzimuth = getAbsoluteAzimuth();

        if(primaryAzimuth<currentAzimuth){
            secondaryAzimuth = primaryAzimuth-180;
        }else{
            secondaryAzimuth = primaryAzimuth +180;
        }

        return secondaryAzimuth;
    }
}