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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
  

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


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
  int closest = 600; 
  SerialPort arduino;
  final int leftMax = 154;
  final int rightMax = 162;
  final int min = 0;  
  final double visFastSpeed = .75;
  final double visSlowSpeed = .5;
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
  DoubleSolenoid MOAC; // mother of all cylinders
  AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  WPI_TalonSRX climbTalon = new WPI_TalonSRX(12);
  WPI_VictorSPX climbFollower = new WPI_VictorSPX(13);  
  int climbSpeed = 1; // value to set the climb talons to
  float climbAngle = 45;  
  PowerDistributionPanel pdp = new PowerDistributionPanel();
  
  //*** PNEUMATICS ***  
  Compressor compress;  

  DoubleSolenoid hatchHolder = new DoubleSolenoid (0,1);
  boolean hatchState = false;

  DoubleSolenoid cargoHolder = new DoubleSolenoid (2,3);
  boolean cargoState = false;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    try {
      compress = new Compressor(0);
      compress.setClosedLoopControl(true);
    } catch (Exception compressorException) {
      System.out.println("Well shoot, compressor startup failed");
      compress = null;
    }
      //*** ARDUINO SERIAL PORT FINDER ***
      try {
        SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kUSB);
        System.out.println("Connected to kUSB");
      } catch (Exception e) {
        System.out.println("Couldn't connect to kUSB, trying kUSB1");
        try {
          SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kUSB2);
          System.out.println("Connected to kUSB2");
        } catch (Exception e2) {
          System.out.println("Not connected to any of the USB ports, trying MXP spot");
          try {
            SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kMXP);
            System.out.println("Connected to MXP port");
          } catch (Exception eMXP) {
            System.out.println("Not Connected to MXP port, trying Onboard");
            try {
              SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kOnboard);
              System.out.println("Connected to Onboard");
            } catch (Exception eOnboard) {
              System.out.println("Not connected to any ports on the RoboRIO");
              arduino = null;
            } // catch (Exception eOnboard)
          }
        }
    }
    /**
     * Primary parser: Other methods are for legacy reasons and for edge cases
     */
    } 
  @Override
  public void autonomousInit() {
  }
  @Override
  public void autonomousPeriodic() {
  }
  @Override
  public void teleopInit() {
     climbTalon.setNeutralMode(NeutralMode.Brake);
     }
   
  @Override
  public void teleopPeriodic() { 
    chassisDrive.arcadeDrive(forward, turn);
    //Pneumatics Code
    if (copilotStick.getRawButtonPressed(4)){ //Hatch toggle
      if (hatchState){
        hatchState = false;
        hatchHolder.set(Value.kForward); //Release the thingamabob
      }
      if (!hatchState){
        hatchState=true;
        hatchHolder.set(Value.kReverse); //Grab the annoying thing
      }  
    }
    if (copilotStick.getRawButtonPressed(5)) {
      if (cargoState){
        cargoState = false;
        cargoHolder.set(Value.kForward);
      }
      if (!cargoState) {
        cargoState = true;
        cargoHolder.set(Value.kReverse);
      }
    } 
    if (copilotStick.getRawButton(1)) { //floor level elevator
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(0));                        
    }
    if (copilotStick.getRawButton(4)) { //cargo rocket lvl 1
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(24));                              
    }
    if (copilotStick.getRawButton(1)) { //cargo rocket lvl 2
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(55));
    }
    if (copilotStick.getRawButton(1)) { //cargo rockegt lvl 3
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(83));
    }
    if (copilotStick.getRawButton(1)) { //cargo cargo ship
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(43));
    }    
    if (copilotStick.getRawButton(1)) { //hatch rocket lvl 1 / std hatch height
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(20));
    }
    if (copilotStick.getRawButton(1)) { //hatch rocket lvl 2 
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(48));     
    }
    if (copilotStick.getRawButton(1)){//Hatch rocket lvl 3
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(77));      
    }
    if (copilotStick.getRawButton(7)) { //outtake
    }
    if (copilotStick.getRawButton(8)) { //rotate intake out
    }
    if (copilotStick.getRawButton(9)) { //rotate intake in
    }
    // ***CLIMB***
    if (leftStick.getRawButton(1)) { //climb
    // if the talon stalls, as indicated by drawing too much current, put it in brake mode
      if (pdp.getCurrent(12) < 10) { //tune this value (10)
climbTalon.set(climbSpeed); // tune this value
      }
      else {climbTalon.set(0);}
      // if the robot is at or greater than a predetermined angle, fire the MOAC.
      if (gyro.getPitch() > climbAngle) 
      {MOAC.set(DoubleSolenoid.Value.kForward);}
    }
    if (leftStick.getRawButton(2)) { //retract piston
      MOAC.set(DoubleSolenoid.Value.kReverse);
    }        
    if (rightStick.getRawButton(1)) { //Vision tracking (line up the robot to the target)
      String targetPosition = arduino.readString();
      int startOfDataStream = targetPosition.indexOf("B");
      int endOfDataStream = targetPosition.indexOf("\r");// looking for the first carriage return
      // The indexOf method returns -1 if it can't find the char in the string
      if (startOfDataStream != -1 && endOfDataStream != -1 && (endOfDataStream - startOfDataStream) > 12) {
        targetPosition = (targetPosition.substring(startOfDataStream, endOfDataStream));
        System.out.println(targetPosition);
        if (targetPosition.startsWith("Block")) {
          String[] positionNums = targetPosition.split(":");
          // positionNums[0] would be "Block
          // positionNums [1] would be number of block: always 0
          xVal = Integer.parseInt(positionNums[2]);
          yVal = Integer.parseInt(positionNums[3]);
          wVal = Integer.parseInt(positionNums[4]);
          hVal = Integer.parseInt(positionNums[5]);
          distVal = Integer.parseInt(positionNums[6]);
          confVal = Integer.parseInt(positionNums[7]);
          blocksSeen = Integer.parseInt(positionNums[1]);
          arduinoCounter = Integer.parseInt(positionNums[8]);
        } else {
          System.out.println("Bad String from Arduino: Doesn't start with Block");
        }
      } else {
        System.out.println("Bad String from Arduino: no carriage return character or too short");
      }
      if (distVal > closest) {
      if (xVal > leftMax && xVal < rightMax) {
        leftSide.set(visFastSpeed);
        rightSide.set(visFastSpeed);        
      } else if (xVal < leftMax) {
        leftSide.set(visSlowSpeed);
        rightSide.set(visFastSpeed);
      }
      else if (xVal > rightMax) {
        leftSide.set(visFastSpeed);
        rightSide.set(visSlowSpeed);
      }
    }
  }
}                      
  public int convertElevatorHeightToNativeUnits(double height){
    return (int) (height / (2*(Math.PI)) *4096 / elevatorGearRatio);
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

  @Override
  public void disabledInit() {
    
  }
}