/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

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
  DriveTrain chassisDrive = new DriveTrain();
  PID testPID = new PID();
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    chassisDrive.arcadeDrive(leftStick.getRawAxis(1), rightStick.getRawAxis(0));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
