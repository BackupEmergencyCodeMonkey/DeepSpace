/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //*** CONSTANTS!! *** (MEASUREMENTS IN METRIC) INTAKE IS VICTORS, REST ARE TALONS
  //*** WHEELS ***
  public static final double wheelDiameter = 15.240; // cm
  public static final double wheelCirc = 2*(Math.PI)*wheelDiameter; // 2(pi)d
  public static final double drivetrainGearRatio = 10.49; // 10.48:1
  //*** ELEVATOR ***
  public static final double hubDiameter = 5.080; // cm
  public static final double hubCirc = 2*(Math.PI)*wheelDiameter; // 2(pi)d
  public static final double elevatorGearRatio = 70; //70:1
  //public static final double elevatorGearRatio = 100; //100:1 COMPBOT RATIO
  //*** CLIMBER ***
  public static final double climberGearRatio = 90; // 90:1
  public static final double MOACActivateAngle = 40.000; //40 DEGREES
  //*** ARM ***
  public static final double ElbowGearRatio = 100; //100:1
  public static final double wristGearRatio = 100; //100:1
  public static final double intakeGearRatio = 10; //10:1
  //*** FIELD HEIGHT *** IN CM (TO CENTERS)
  public static final double lowRocketCargo = 28;
  public static final double midRocketCargo = 55;
  public static final double highRocketCargo = 83;
  public static final double lowHatch = 20;
  public static final double midHatch = 48;
  public static final double highHatch = 77;
  public static final double cargoBay = 43;
  
  Joystick leftStick = new Joystick(0);
  Joystick rightStick = new Joystick(1);
  Joystick copilotStick = new Joystick(2);
  WPI_TalonSRX motor;
  WPI_TalonSRX elevatorTalon = new WPI_TalonSRX(9);
  //*** DRIVETRAIN ***
  WPI_TalonSRX FrontLeft = new WPI_TalonSRX(1);
  WPI_VictorSPX MiddleLeft = new WPI_VictorSPX(2);
  WPI_VictorSPX BackLeft = new WPI_VictorSPX(3);
  WPI_TalonSRX   FrontRight = new WPI_TalonSRX(4);
  WPI_VictorSPX MiddleRight = new WPI_VictorSPX(5);
  WPI_VictorSPX BackRight = new WPI_VictorSPX(6);
  SpeedControllerGroup leftSide = new SpeedControllerGroup(FrontLeft, MiddleLeft, BackLeft);
  SpeedControllerGroup rightSide = new SpeedControllerGroup(FrontRight, MiddleRight, BackRight);
  DifferentialDrive chassisDrive = new DifferentialDrive(leftSide, rightSide);

 //*** VISION ***
  int bRate;
  int counting;
  int xVal; //In order of appearance
  int yVal;
  int wVal;
  int hVal;
  int distVal;
  int confVal;
  int stopDistance;
  int confidenceThreshold;
  int delayCount;
  int arduinoCounter; // loop counter passed from arduino for timing checks
  int startOfDataStream;
  int endOfDataStream;
  int blocksSeen;
  SerialPort arduino;
  final int leftMax = 154;
  final int rightMax = 162;
  final int min = 0;
  //*** PID ***
  public double P = 1.0;
  public double I = 0.0;
  public double D = 0.0;
  public double F = 0.0;
  public int slot = 0;
//*** JOYSTICKS ***
  double forward = leftStick.getRawAxis(1);
  double turn = rightStick.getRawAxis(0);
  //*** CLIMB ***
  WPI_VictorSPX climbFollower;
  DoubleSolenoid MOAC; // mother of all cylinders
  AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  WPI_TalonSRX climbTalon = new WPI_TalonSRX(12);
WPI_VictorSRX climbFollower = new WPI_VictorSPX(13);  
  int climbTarget = 5; // value to set the climb talons to
  double climbAngle = 45;  
  //*** PNEUMATICS ***
  Compressor compress;  

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    try {
      compress = new Compressor(0);
      compress.setClosedLoopControl(true);

  } catch (Exception e) {
      System.out.println("Well shoot, compressor startup failed");
      compress = null;
  }

            
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    chassisDrive.arcadeDrive(forward, turn);

    //Pneumatics Code
    if (copilotStick.getRawButton(1))
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    motor = elevatorTalon;
  }

  @Override
  public void testPeriodic() {    
    motor.config_kP(slot, P);
    motor.config_kI(slot, I);
    motor.config_kD(slot, D);
    motor.config_kF(slot, F);
  }

}