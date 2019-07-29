package frc.util;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

/**
 * Benzene Bot representation for Rev SPARK MAX
 */
public class BbSparkMax extends CANSparkMax {

    private CANPIDController pidController;

    public BbSparkMax(int deviceId, CANSparkMaxLowLevel.MotorType type) {
        super(deviceId, type);
        pidController = getPIDController();
    }
    /**
     * Set PID to be activated
     *
     * @param value value of unit depending on control type
     * @param ctrl Control Type (Position, Velocity etc.)
     */
    public void setPIDReference(double value, ControlType ctrl){
        pidController.setReference(value, ctrl);
    }
    
}
