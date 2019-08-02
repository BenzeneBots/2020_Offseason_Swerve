package frc.systems.arcadedrive.toplevel;

import edu.wpi.first.wpilibj.Joystick;
import frc.systems.arcadedrive.lowerlevel.ArcadeDriveLowerLevel;
import frc.systems.arcadedrive.toplevel.config.ArcadeConfig;

public class ArcadeDrive extends ArcadeDriveLowerLevel{
    
    /**
     * Instantiates the drivetrain motor contollers, inverts and sets sensor phase and configures feedback devices. Also
     * prepares motor controllers for 10ms period Motion Profile Curves.
     */
    public ArcadeDrive(){
        super();
    }

    public void runArcade(Joystick driveJoy){
        double throttle = 0.0;
        double steer = 0.0;

        
        throttle = powerInput(this.driveRevFactor * -1.0 * driveJoy.getY(ArcadeConfig.THROTTLE_AXIS),2);
        steer = 0.40 * powerInput(driveJoy.getRawAxis(4),2);

        this.reverseDrive(driveJoy.getRawButtonPressed(ArcadeConfig.REVERSE_BUTTON));

        DriveSpeed driveSpeed = calculateSpeed(throttle, steer);
        setDriveMotorScaledSpeed(driveSpeed);
    }
}