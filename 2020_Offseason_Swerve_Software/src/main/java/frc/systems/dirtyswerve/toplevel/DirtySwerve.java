/**
 * @author SanketNayak2019
 */

package frc.systems.dirtyswerve.toplevel;

import edu.wpi.first.wpilibj.Joystick;
import frc.systems.dirtyswerve.toplevel.config.SwerveConfig;
import frc.systems.dirtyswerve.toplevel.config.SwerveConfig.DriveMode;
import frc.systems.dirtyswerve.lowerlevel.DirtySwerveLowerLevel;

public class DirtySwerve extends DirtySwerveLowerLevel{

  /**
   * This is the central file to run the swerve drive. Developed based on the inverse kinematic equations developed
   * from a Chief Delphi post authored by Ether ( "https://www.chiefdelphi.com/media/papers/2426" )
   * 
   * All arrays are numbered 0-3 from front to back, with even numbers on the left side when
   * facing forward.
   */
  public DirtySwerve(){
    super();
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

    double x = SwerveConfig.DISPLACEMENT_FACTOR * powerInput( movementJoy.getX() , 2);
    double y = SwerveConfig.DISPLACEMENT_FACTOR * powerInput( movementJoy.getY() , 2);
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

    if (gyro != null && SwerveConfig.SELECTED_DRIVE_MODE == DriveMode.FIELD_ORIENTED) {
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
}