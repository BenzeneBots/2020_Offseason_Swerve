package frc.util;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;

public class PID_Config{

    private double kP, kI, kD, kF;
    private int iZone; 

    /**
     * This class is used to create a PID configuration that can be applied to both the new SparkMax Controllers as well as 
     * talon and victor's controllers. 
     * @param kP Assign a P value for the assosciated PID loop
     * @param kI Assign an I value for the assosciated PID loop
     * @param kD Assign a D value for the assosciated PID loop
     * @param kF Assign a F value for the assosciated PID loop
     * @param iZone Assign a designated zone of integration for the assosciated PID loop
     */
    public PID_Config(double kP,double kI,double kD,double kF,int iZone){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
    }

    /**
     * Fetches the P Value of the PID Configuration
     * @return Returns a constant P value
     */
    public double getP(){
        return this.kP;
    }

    /**
     * Fetches the I Value of the PID Configuration
     * @return Returns a constant I value
     */
    public double getI(){
        return this.kI;
    }

    /**
     * Fetches the D Value of the PID Configuration
     * @return Returns a constant D value
     */
    public double getD(){
        return this.kD;
    }

    /**
     * Fetches the F Value of the PID Configuration
     * @return Returns a constant F value
     */
    public double getF(){
        return this.kF;
    }

    /**
     * Fetches the Integral Zone of the PID Configuration
     * @return Returns a constant Integral Zone
     */ 
    public int getIntegralZone(){
        return this.iZone;
    }

    /**
     * Assigns the PID configuration to a Talon Motor Controller
     * @param motorController Talon SRX motor controller object
     */
    public void assign(WPI_TalonSRX motorController){
        this.assign(0,motorController);
    }

    /**
     * Assigns the PID configuration to a Talon Motor Controller
     * @param pidSlot Selected PID priority slot (0 is default)
     * @param motorController Talon SRX motor controller object
     */
    public void assign(int pidSlot,WPI_TalonSRX motorController){
        motorController.config_kP(pidSlot, this.kP);
        motorController.config_kI(pidSlot, this.kI);
        motorController.config_kD(pidSlot, this.kD);
        motorController.config_kF(pidSlot, this.kF);
        motorController.config_IntegralZone(pidSlot, this.iZone);
    }

    /**
     * Assigns the PID configuration to a VictorSPX Motor Controller
     * @param motorController Victor motor controller object
     */
    public void assign(WPI_VictorSPX motorController){
        this.assign(0,motorController);
    }

    /**
     * Assigns the PID configuration to a VictorSPX Motor Controller
     * @param pidSlot Selected PID priority slot (0 is default)
     * @param motorController Victor motor controller object
     */
    public void assign(int pidSlot,WPI_VictorSPX motorController){
        motorController.config_kP(pidSlot, this.kP);
        motorController.config_kI(pidSlot, this.kI);
        motorController.config_kD(pidSlot, this.kD);
        motorController.config_kF(pidSlot, this.kF);
        motorController.config_IntegralZone(pidSlot, this.iZone);

    }

    /**
     * Assigns the PID configuration to a SparkMax Motor Controller
     * @param motorController Spark max motor controller object
     */
    public void assign(CANSparkMax motorController){
        motorController.getPIDController().setP(this.kP);
        motorController.getPIDController().setI(this.kI);
        motorController.getPIDController().setD(this.kD);
        motorController.getPIDController().setFF(this.kF);
        motorController.getPIDController().setIZone(this.iZone);
    }

}