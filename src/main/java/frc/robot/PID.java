package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class PID {
  private double P;
  private double I;
  private double D;
  private double F;
  private int slot;
  private WPI_TalonSRX motor;

  public PID(WPI_TalonSRX driver, double pVal, double iVal, double dVal, double fVal, int slot) {
    P = pVal;
    I = iVal;
    D = dVal;
    F = fVal;
    this.slot = slot;
    motor = driver;
    motor.config_kP(slot, P);
    motor.config_kI(slot, I);
    motor.config_kD(slot, D);
    motor.config_kF(slot, F);
  }
}