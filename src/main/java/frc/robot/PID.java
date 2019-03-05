package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class PID {
  private double P;
  private double I;
  private double D;
  private double F;
  private int slot;
  private WPI_TalonSRX motor;
  
  public PID() {
  slot = 0;
  P = 0;
  I = 0;
  D = 0;
  F = 0;
  }
  /**
  *  @param slot the desired 'PID Slot' value, 0 - 3 expected. Think profiles.
  */ 
/**
 * 
 * @param driver the talon in use.
 * @param pVal the desired 'Proportional' or P value.
 * @param iVal the desired 'Integral' or I value.
 * @param dVal the desired 'Derivitive' or D value.
 * @param fVal the desired 'Feed Forward' or F value.
 * @param slot the desired 'PID Slot' value, 0 - 3 expected. Think profiles.
 */
  public PID(WPI_TalonSRX driver, double pVal, double iVal, double dVal, double fVal, int slot) {
    P = pVal;
    I = iVal;
    D = dVal;
    F = fVal;
    this.slot = slot;
    motor = driver;
  }
  public double getP() {
    return P;
  }
  public double getI() {
    return I;
  }
  public double getD() {
    return D;
  }
  public double getF() {
    return F;
  }
  public int getSlot() {
    return slot;
  }
  public WPI_TalonSRX getMotor() {
    return motor;
  }
  public void setP(double newP) {
    P = newP;
  }
  public void setI(double newI) {
    I = newI;
  }
  public void setD(double newD) {
    D = newD;
  }
  public void setF(double newF) {
    F = newF;
  }
  public void setSlot(int newSlot) {
    slot = newSlot;
  }
  public void setMotor(WPI_TalonSRX newMotor) {
    motor = newMotor;
  }
  public void useAll() {
    motor.config_kP(slot, P);
    motor.config_kI(slot, I);
    motor.config_kD(slot, D);
    motor.config_kF(slot, F);
  }
  public void useP() {
    motor.config_kP(slot, P);
  }
  public void useI() {
    motor.config_kI(slot, I);
  }
  public void useD() {
    motor.config_kD(slot, D);
  }
  public void useF() {
    motor.config_kD(slot, F);
  }
}