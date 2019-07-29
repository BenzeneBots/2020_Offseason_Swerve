/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.dirtyswerve.DirtySwerve;

public class Robot extends TimedRobot {
  
  private DirtySwerve drive = new DirtySwerve();
  private Joystick driveJoy1 = new Joystick(1);
  private Joystick driveJoy2 = new Joystick(2);

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drive.teleopControl(driveJoy1, driveJoy2);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
