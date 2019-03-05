package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
public class DriveTrain {
  private WPI_TalonSRX FrontLeft;
  private WPI_VictorSPX MiddleLeft;
  private WPI_VictorSPX BackLeft;
  private WPI_TalonSRX FrontRight;
  private WPI_VictorSPX MiddleRight;
  private WPI_VictorSPX BackRight;
  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide;
  DifferentialDrive drive;
  public DriveTrain() {
    FrontLeft = new WPI_TalonSRX(1);
    MiddleLeft = new WPI_VictorSPX(2);
    BackLeft = new WPI_VictorSPX(3);
    FrontRight = new WPI_TalonSRX(4);
    MiddleRight = new WPI_VictorSPX(5);
    BackRight = new WPI_VictorSPX(6);
    leftSide = new SpeedControllerGroup(FrontLeft, MiddleLeft, BackLeft);
    rightSide = new SpeedControllerGroup(FrontRight, MiddleRight, BackRight);
    drive = new DifferentialDrive(leftSide, rightSide);
}
  public DriveTrain(int FL, int ML, int BL, int FR, int MR, int BR) {
    FrontLeft = new WPI_TalonSRX(FL);
    MiddleLeft = new WPI_VictorSPX(ML);
    BackLeft = new WPI_VictorSPX(BL);
    FrontRight = new WPI_TalonSRX(FR);
    MiddleRight = new WPI_VictorSPX(MR);
    BackRight = new WPI_VictorSPX(BR);
    leftSide = new SpeedControllerGroup(FrontLeft, MiddleLeft, BackLeft);
    rightSide = new SpeedControllerGroup(FrontRight, MiddleRight, BackRight);
    drive = new DifferentialDrive(leftSide, rightSide);
}
  public DriveTrain(WPI_TalonSRX FL, WPI_VictorSPX ML, WPI_VictorSPX BL, WPI_TalonSRX FR, WPI_VictorSPX MR, WPI_VictorSPX BR) {
    FrontLeft = FL;
    MiddleLeft = ML;
    BackLeft = BL;
    FrontRight = FR;
    MiddleRight = MR;
    BackRight = BR;
    leftSide = new SpeedControllerGroup(FrontLeft, MiddleLeft, BackLeft);
    rightSide = new SpeedControllerGroup(FrontRight, MiddleRight, BackRight);
    drive = new DifferentialDrive(leftSide, rightSide);
}
  public DriveTrain(SpeedControllerGroup L, SpeedControllerGroup R) {
    leftSide = L;
    rightSide = R;
    drive = new DifferentialDrive(leftSide, rightSide);
}
  public void arcadeDrive(double forward, double turn) {
  drive.arcadeDrive(forward, turn);
  }
}