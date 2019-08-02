package frc.systems.arcadedrive.lowerlevel;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.systems.arcadedrive.toplevel.config.ArcadeConfig;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

public class ArcadeDriveLowerLevel{
    private WPI_TalonSRX leftMaster;
    private WPI_VictorSPX leftSlave;

    private WPI_TalonSRX rightMaster;
    private WPI_VictorSPX rightSlave;

    public static boolean isDriveReversed;
    protected double driveRevFactor = 1.0;

    /**
     * Instantiates the drivetrain motor contollers, inverts and sets sensor phase and configures feedback devices. Also
     * prepares motor controllers for 10ms period Motion Profile Curves.
     */
    public ArcadeDriveLowerLevel(){
        leftMaster = new WPI_TalonSRX(ArcadeConfig.LEFT_MASTER_ID);
        leftSlave = new WPI_VictorSPX(ArcadeConfig.LEFT_SLAVE_ID);

        rightMaster = new WPI_TalonSRX(ArcadeConfig.RIGHT_MASTER_ID);
        rightSlave = new WPI_VictorSPX(ArcadeConfig.RIGHT_SLAVE_ID);

        motorControllerConfig();
    }

    /**
     * Inverts and sets sensor phase and configures feedback devices. Also
     * prepares motor controllers for 10ms period Motion Profile Curves.
     */
    private void motorControllerConfig(){
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());
        leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());

        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);

        sensorConfig();

        leftMaster.configOpenloopRamp(ArcadeConfig.OPEN_LOOP_RAMP_RATE);
        rightMaster.configOpenloopRamp(ArcadeConfig.OPEN_LOOP_RAMP_RATE);

        rightMaster.configClosedloopRamp(ArcadeConfig.CLOSED_LOOP_RAMP_RATE);
        leftMaster.configClosedloopRamp(ArcadeConfig.CLOSED_LOOP_RAMP_RATE);

        rightMaster.configNeutralDeadband(0.01);
        leftMaster.configNeutralDeadband(0.01);
    }

    /**
     * Configures sensors with PID, trajectory rates of 10ms, and set control frame periods
     */
    private void sensorConfig(){
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        rightMaster.setSensorPhase(true);
        leftMaster.setSensorPhase(true);

        ArcadeConfig.DRIVE_PID.assign(leftMaster);
        ArcadeConfig.DRIVE_PID.assign(rightMaster);

        rightMaster.configMotionProfileTrajectoryPeriod(10);
        leftMaster.configMotionProfileTrajectoryPeriod(10);

        rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);

        rightMaster.changeMotionControlFramePeriod(5);
        leftMaster.changeMotionControlFramePeriod(5);

        rightMaster.clearMotionProfileTrajectories();
        leftMaster.clearMotionProfileTrajectories();
    }

    /**
     * Updates direction of driving based on button input
     * @param buttonPress an instantaneous get(Raw)ButtonPressed input that is used as a toggle
     */
    protected void reverseDrive(boolean buttonPress){
        if(buttonPress){
            driveRevFactor *= -1.0;
            isDriveReversed = !isDriveReversed;
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

    /**
     * Quickly set and bound speed for all four motors on the drivetrain.
     *
     * @param mtrLeftSp  Percent Output for Left Drivetrain
     * @param mtrRightSp Percent Output for Right Drivetrain
     */
    public void setDriveMotorScaledSpeed(DriveSpeed driveSpeed) {
        driveSpeed.scaleDriveOutput(ArcadeConfig.MAX_SPEED);
        leftMaster.set(driveSpeed.getLeftSpeed());
        rightMaster.set(driveSpeed.getRightSpeed());
    }

    /**
     * Instantly sets drivetrain speeds to 0% output.
     */
    public void stopDrive(){
        setDriveMotorScaledSpeed(new DriveSpeed(0,0));
    } 

    /**
     * Determines relative speeds for left and right side of the drivetrain based on throttle and steer input
     *
     * @param throttle The output from joysticks for forward driving
     * @param steer    Scaled joystick output for turning
     */
    protected DriveSpeed calculateSpeed(double throttle, double steer) {
        return new DriveSpeed(throttle + steer, throttle - steer);
    }

    public static class DriveSpeed {
        private double leftSpeed;
        private double rightSpeed;

        public DriveSpeed(double leftSpeed, double rightSpeed) {
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
        }

        /**
         * Determines if the asked output speed is greater than the allocated max speed
         * and if either is over, scales both speeds down to allow for optimal turning
         *
         * @param maxSpeed scaling factor
         */
        protected DriveSpeed scaleDriveOutput(double maxSpeed) {
            double conversionFactor = 1;

            if (Math.abs(leftSpeed) > maxSpeed) {
                conversionFactor = Math.abs(maxSpeed / leftSpeed);

            } else if (Math.abs(rightSpeed) > maxSpeed) {
                conversionFactor = Math.abs(maxSpeed / rightSpeed);
            }
            leftSpeed *= conversionFactor;
            rightSpeed *= conversionFactor;

            return this;
        }

        public double getLeftSpeed() {
            return leftSpeed;
        }

        public double getRightSpeed() {
            return rightSpeed;
        }
    }
}