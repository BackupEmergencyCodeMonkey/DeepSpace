package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
 /** arms turn, then after the robot reaches a certain angle, the cylinder activate.
     *  One method to begin climb,
     * one method to unclimb
     */
public class Climb {

    WPI_VictorSPX climbFollower;
    DoubleSolenoid MOAC; // mother of all cylinders
    AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    WPI_TalonSRX talon;
    int target = 5; // value to set the climb talons to

/**@param port the port number of the main climber talon (12)
 * @param victorPort the port number of the victor that should follow this (calling this 13 for now)
 * @param cylPort1 the forward channel number of the Mother of All Cylinders (our giant climbing cylinder)
 * @param cylPort2 the reverse channel number of the MOAC
 */
   public Climb (int port, int victorPort, int cylPort1, int cylPort2) { // this season, the port is 12, and the portVictor is 
        talon = new WPI_TalonSRX(port);
       climbFollower = new WPI_VictorSPX(victorPort);
        climbFollower.set(ControlMode.Follower, port);
        MOAC = new DoubleSolenoid(cylPort1, cylPort2);

} // edit these numbers to tune
   // 12 is the port number of the climber

/**sets the climber arms to rotate out to a certain angle. Senses  then turns the MOAC on */
   public void climb(float angle) {
        talon.set(target); 
    //if () // put a NavX angle of the robot test here
        if (gyro.getPitch() > angle)  // tune this. Pitch is the most likely axis 
       MOAC.set(DoubleSolenoid.Value.kForward);
   }

   public void unClimb() {
       talon.set(-target);
       MOAC.set(DoubleSolenoid.Value.kReverse);
    }

}