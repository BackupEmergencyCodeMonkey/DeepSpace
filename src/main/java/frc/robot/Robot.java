/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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

  Vision vis = new Vision(115280);
  public double P = 1.0;
  public double I = 0.0;
  public double D = 0.0;
  public double F = 0.0;
  public int slot = 0;
  double forward = leftStick.getRawAxis(1);
  double turn = rightStick.getRawAxis(0);
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    vis.getArduino();
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