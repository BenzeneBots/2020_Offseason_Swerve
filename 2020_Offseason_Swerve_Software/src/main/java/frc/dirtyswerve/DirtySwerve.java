/**
 * @author SanketNayak2019
 */

package frc.dirtyswerve;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import frc.dirtyswerve.util.SwerveUtil;
import frc.dirtyswerve.util.SwerveUtil.DriveMode;
import frc.dirtyswerve.module.SwerveModule;

public class DirtySwerve{
    
  private SwerveModule frontLeftModule,frontRightModule,rearLeftModule,rearRightModule;
  private PigeonIMU gyro;

  /**
   * This is the central file to run the swerve drive. Developed based on the inverse kinematic equations developed
   * from a Chief Delphi post authored by Ether ( "https://www.chiefdelphi.com/media/papers/2426" )
   * 
   * All arrays are numbered 0-3 from front to back, with even numbers on the left side when
   * facing forward.
   */
  public DirtySwerve(){
    gyro = new PigeonIMU(SwerveUtil.GYRO_ID);

    frontLeftModule = new SwerveModule("FrontLeftModule", SwerveUtil.FRONT_LEFT_THROTTLE, SwerveUtil.FRONT_LEFT_AZIMUTH);
    frontRightModule = new SwerveModule("FrontRightModule", SwerveUtil.FRONT_RIGHT_THROTTLE, SwerveUtil.FRONT_RIGHT_AZIMUTH);
    rearLeftModule = new SwerveModule("RearLeftModule", SwerveUtil.REAR_LEFT_THROTTLE, SwerveUtil.REAR_LEFT_AZIMUTH);
    rearRightModule = new SwerveModule("RearRightModule", SwerveUtil.REAR_RIGHT_THROTTLE, SwerveUtil.REAR_RIGHT_AZIMUTH);
  }

  /**
   * Assumes all the azimuths of the modules are 0. Should only need to be calibrated if the encoder is moved.
   */
  public void recalibrateAzimuths(){
    frontLeftModule.resetOffset();
    frontRightModule.resetOffset();
    rearLeftModule.resetOffset();
    rearRightModule.resetOffset();
  }
  
  /**
   * Runs swerve drive based on the input of controllers. This class should be customized for the driver.
   * @param movementJoy This joystick controls the forward and strafe axes.
   * @param rotationJoy This joystick controls the rotation of the robot.
   */
  public void teleopControl(Joystick movementJoy, Joystick rotationJoy){

    double x = SwerveUtil.DISPLACEMENT_FACTOR * powerInput( movementJoy.getX() , 2);
    double y = SwerveUtil.DISPLACEMENT_FACTOR * powerInput( movementJoy.getY() , 2);
    double t =+ 0.5*powerInput(rotationJoy.getTwist(),2);
    t = Math.IEEEremainder(t, 2);

    drive(y, x, t);
  }

  /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double azimuth) {
    double angle=0;

    if (gyro != null && SwerveUtil.SELECTED_DRIVE_MODE == DriveMode.FIELD_ORIENTED) {
      angle = gyro.getFusedHeading();
      angle = Math.IEEEremainder(angle, 360.0);
    }

    angle = Math.toRadians(angle);
    strafe = -forward * Math.sin(angle) + strafe * Math.cos(angle);
    forward = forward * Math.cos(angle) + strafe * Math.sin(angle);

    double[] trajVars = calcTrajVars(forward, strafe, azimuth);

    double[] spds = calcWheelSpds(trajVars);

    double[] azms = calcWheelAzms(trajVars);

    feedModules(spds, azms);
  }

  /**
   * Calculates the variables used to generate speeds and azimuths as an array of 4. Based off of
   * the letter (a,b,c,d) variables described in Ether's Chief Delphi Post.
   * @param forward Net Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe Net X-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param azimuth Absolute robot rotation from -1.0 CCW to 1.0 CW
   * @return Returns a double array with the variables stored
   */
  private double[] calcTrajVars(double forward, double strafe, double azimuth){
    double[] vars = new double [4];

    double radius = Math.hypot(SwerveUtil.CHASSIS_LENGTH, SwerveUtil.CHASSIS_WIDTH);
    double relativeLength = SwerveUtil.CHASSIS_LENGTH/radius;
    double relativeWidth = SwerveUtil.CHASSIS_WIDTH/radius;

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
  private double[] calcWheelSpds(double[] trajVars){
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
  private double[] normalizeSpds(double[] speeds){
    final double maxWheelSpeed = this.getMaxSpeed(speeds);

    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < 4; i++) {
        speeds[i] /= maxWheelSpeed;
      }
    }

    return speeds;
  }

  private double getMaxSpeed(double[] speeds){
    return Math.max(Math.max(speeds[0], speeds[1]), Math.max(speeds[2], speeds[3]));
  }

  private double[] calcWheelAzms(double[] trajVars){
    double[] azms = new double[4];

    azms[0] = Math.atan2(trajVars[1], trajVars[3]) * 180 / Math.PI;
    azms[1] = Math.atan2(trajVars[1], trajVars[2]) * 180 / Math.PI;
    azms[2] = Math.atan2(trajVars[0], trajVars[3]) * 180 / Math.PI;
    azms[3] = Math.atan2(trajVars[0], trajVars[2]) * 180 / Math.PI;

    return azms; 
  }

  private void feedModules(double[] spds,double[] azms){
    //does not update azimuths if the speed is under threshold
    if(this.getMaxSpeed(spds)>SwerveUtil.MIN_THROTTLE){ 
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
    private double powerInput(double input, int degree) {
      if (input >= 0) {
          return Math.pow(input, degree);
      } else {
          return -Math.abs(Math.pow(input, degree));
      }
  }

}


