/**
 * @author SanketNayak2019
 */

package frc.systems.dirtyswerve.lowerlevel;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.systems.dirtyswerve.toplevel.config.SwerveConfig;
import frc.systems.dirtyswerve.lowerlevel.module.SwerveModule;

public class DirtySwerveLowerLevel{
    
  protected SwerveModule frontLeftModule,frontRightModule,rearLeftModule,rearRightModule;
  protected PigeonIMU gyro;

  /**
   * This is the central file to run the swerve drive. Developed based on the inverse kinematic equations developed
   * from a Chief Delphi post authored by Ether ( "https://www.chiefdelphi.com/media/papers/2426" )
   * 
   * All arrays are numbered 0-3 from front to back, with even numbers on the left side when
   * facing forward.
   */
  public DirtySwerveLowerLevel(){
    gyro = new PigeonIMU(SwerveConfig.GYRO_ID);

    frontLeftModule = new SwerveModule("FrontLeftModule", SwerveConfig.FRONT_LEFT_THROTTLE_ID, SwerveConfig.FRONT_LEFT_AZIMUTH_ID);
    frontRightModule = new SwerveModule("FrontRightModule", SwerveConfig.FRONT_RIGHT_THROTTLE_ID, SwerveConfig.FRONT_RIGHT_AZIMUTH_ID);
    rearLeftModule = new SwerveModule("RearLeftModule", SwerveConfig.REAR_LEFT_THROTTLE_ID, SwerveConfig.REAR_LEFT_AZIMUTH_ID);
    rearRightModule = new SwerveModule("RearRightModule", SwerveConfig.REAR_RIGHT_THROTTLE_ID, SwerveConfig.REAR_RIGHT_AZIMUTH_ID);
  }

  /**
   * Calculates the variables used to generate speeds and azimuths as an array of 4. Based off of
   * the letter (a,b,c,d) variables described in Ether's Chief Delphi Post.
   * @param forward Net Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe Net X-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param azimuth Absolute robot rotation from -1.0 CCW to 1.0 CW
   * @return Returns a double array with the variables stored
   */
  protected double[] calcTrajVars(double forward, double strafe, double azimuth){
    double[] vars = new double [4];

    double radius = Math.hypot(SwerveConfig.CHASSIS_LENGTH, SwerveConfig.CHASSIS_WIDTH);
    double relativeLength = SwerveConfig.CHASSIS_LENGTH/radius;
    double relativeWidth = SwerveConfig.CHASSIS_WIDTH/radius;

    vars[0] = strafe - azimuth * relativeLength;
    vars[1] = strafe + azimuth * relativeLength;
    vars[2] = forward - azimuth * relativeWidth;
    vars[3] = forward + azimuth * relativeWidth;

    return vars;
  }

  /**
   * Returns wheel speeds as an array based on precalculated variables []
   * @param trajVars Precalculated variable dependecies
   * @return 
   */
  protected double[] calcWheelSpds(double[] trajVars){
    double[] speeds = new double[4];

    speeds[0]= Math.hypot(trajVars[1], trajVars[3]);
    speeds[1] = Math.hypot(trajVars[1], trajVars[2]);
    speeds[2] = Math.hypot(trajVars[0], trajVars[3]);
    speeds[3] = Math.hypot(trajVars[0], trajVars[2]);

    speeds = normalizeSpds(speeds);
    return speeds;
  }

  /**
   * Scales down all speed factors to a range of 0-1.
   * @param speeds
   * @return
   */
  protected double[] normalizeSpds(double[] speeds){
    final double maxWheelSpeed = this.getMaxSpeed(speeds);

    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < 4; i++) {
        speeds[i] /= maxWheelSpeed;
      }
    }

    return speeds;
  }

  protected double getMaxSpeed(double[] speeds){
    return Math.max(Math.max(speeds[0], speeds[1]), Math.max(speeds[2], speeds[3]));
  }

  protected double[] calcWheelAzms(double[] trajVars){
    double[] azms = new double[4];

    azms[0] = Math.atan2(trajVars[1], trajVars[3]) * 180 / Math.PI;
    azms[1] = Math.atan2(trajVars[1], trajVars[2]) * 180 / Math.PI;
    azms[2] = Math.atan2(trajVars[0], trajVars[3]) * 180 / Math.PI;
    azms[3] = Math.atan2(trajVars[0], trajVars[2]) * 180 / Math.PI;

    return azms; 
  }

  protected void feedModules(double[] spds,double[] azms){
    //does not update azimuths if the speed is under threshold
    if(this.getMaxSpeed(spds)>SwerveConfig.MIN_THROTTLE){ 
      frontLeftModule.set(azms[0], spds[0]);
      frontRightModule.set(azms[1], spds[1]);
      rearLeftModule.set(azms[2], spds[2]);
      rearRightModule.set(azms[3], spds[3]);
    }
  }

  /**
     * Apply a Exponent to any Input that returns negative if the input is negative and vice-versa
     *
     * @param input  Parameter to Apply the exponent to
     * @param degree Apply this exponent to the input
     * @return Input to the power of the degree
     */
  protected double powerInput(double input, int degree) {
      if (input >= 0) {
          return Math.pow(input, degree);
      } else {
          return -Math.abs(Math.pow(input, degree));
      }
  }

}


