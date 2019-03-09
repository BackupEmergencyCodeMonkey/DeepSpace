/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//TALON NUMBERS, PURPOSE, PDP CHANNEL
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
  

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;


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
  public static final double wheelCirc = (Math.PI)*wheelDiameter; // (pi)d
  public static final double drivetrainGearRatio = 10.49; // 10.48:1
  
  //*** ELEVATOR ***
  public static final double hubDiameter = 2.54; // cm
  public static final double hubCirc = (Math.PI)*hubDiameter; // (pi)d
  //public static final double elevatorGearRatio = 100; //100:1 COMPBOT RATIO

  //*** ELEVATOR AND ARM LENGTHS ***
  public static final double baseArmHeight = (30.0*2.54); //in inches, *2.54 for to CM
  public static final double elbowLength = (16.5*2.54);
  public static final double wristLength = (14.0*2.54);
  
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
  public static final double elevatorLowMidCargoRocketHeight = 42; // halfway between the low and middle cargo heights
  public static final double elevatorHighMidCargoRocketHeight = 69; // halfway between the middle and high cargo heights
  public static final double elevatorLowMidHatchRocketHeight = 34; // halfway between the low and middle cargo heights
  public static final double elevatorHighMidHatchRocketHeight = 61; // halfway between the middle and high cargo heights
  
  WPI_TalonSRX motor;
  WPI_TalonSRX elevatorTalon = new WPI_TalonSRX(9);
  WPI_TalonSRX elbowTalon = new WPI_TalonSRX(10);
  WPI_TalonSRX wristTalon = new WPI_TalonSRX(7);
  //@@@ JW need intake victor assignments
  //WPI_VictorSPX leftIntakeRoller = new WPI_VictorSPX(?);
  //WPI_VictorSPX rightIntakeRoller = new WPI_VictorSPX(?);
/* @@@ JW changing drive train motorcontroller numbers to reflect actual installation
  //*** DRIVETRAIN ***
  WPI_TalonSRX FrontLeft = new WPI_TalonSRX(1);
  WPI_VictorSPX MiddleLeft = new WPI_VictorSPX(2);
  WPI_VictorSPX BackLeft = new WPI_VictorSPX(3);

  WPI_TalonSRX   FrontRight = new WPI_TalonSRX(4);
  WPI_VictorSPX MiddleRight = new WPI_VictorSPX(5);
  WPI_VictorSPX BackRight = new WPI_VictorSPX(6);
 */ 

//*** DRIVETRAIN ***
  WPI_TalonSRX FrontLeft = new WPI_TalonSRX(4);
  WPI_VictorSPX MiddleLeft = new WPI_VictorSPX(5);
  WPI_VictorSPX BackLeft = new WPI_VictorSPX(6);

  WPI_TalonSRX   FrontRight = new WPI_TalonSRX(1);
  WPI_VictorSPX MiddleRight = new WPI_VictorSPX(2);
  WPI_VictorSPX BackRight = new WPI_VictorSPX(3);
  
  SpeedControllerGroup leftSide = new SpeedControllerGroup(FrontLeft, MiddleLeft, BackLeft);
  SpeedControllerGroup rightSide = new SpeedControllerGroup(FrontRight, MiddleRight, BackRight);
  DifferentialDrive chassisDrive = new DifferentialDrive(leftSide, rightSide);

//*** VISION ***
  int bRate = 115200;
  int counting = 0;
  int xVal; //In order of appearance
  int yVal;
  int wVal;
  int hVal;
  int distVal;
  int confVal;
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
  public double DriveP = 0.0;
  public double DriveI = 0.0;
  public double DriveD = 0.0;
  public double DriveF = 0.0;

  public double ElevatorP = 0.0;
  public double ElevatorI = 0.0;
  public double ElevatorD = 0.0;
  // @@@ JW adding an elevator feedforward parameter
  public double ElevatorF = 0.0;
  
  public double ElbowP = 0.0;
  public double ElbowI = 0.0;
  public double ElbowD = 0.0;
  public double ElbowF = 0.0;
  
  public double testP = 0.0;
  public double testI = 0.0;
  public double testD = 0.0;
  public double testF = 0.0;
  
  public int slot = 0;

  //*** JOYSTICKS ***  
  Joystick leftStick = new Joystick(0);
  Joystick rightStick = new Joystick(1);
  Joystick copilotStick = new Joystick(2);
  double forward;
  double turn;
  // *** Buttons ***
  boolean hatchToggle;    
  boolean cargoToggle;
  boolean groundLev;
  boolean cargoLev1;
  boolean cargoLev2;
  boolean cargoLev3;
  boolean hatchLev1;
  boolean hatchLev2;
  boolean hatchLev3;  
  boolean cargoShipLev;
  boolean outtakeButton;
  boolean rotateIntakeOutButton;
  boolean intakeButton;
  boolean climbButton;
  boolean retractClimbCylinderButton;
  boolean visionTrackingButton;
  boolean openHatch;
  boolean closeHatch;
  boolean camSwapButton;
  boolean retractIntake;
  boolean toggleIntake;
        
  
  //*** CLIMB ***
// @@@ JW Asigning solonoid numbers to MOAC  
//  DoubleSolenoid MOAC; // mother of all cylinders
  DoubleSolenoid MOAC = new DoubleSolenoid (0,7); // mother of all cylinders
  AHRS gyro = new AHRS(SerialPort.Port.kMXP);  
  WPI_TalonSRX climbTalon = new WPI_TalonSRX(12);
  WPI_VictorSPX climbFollower = new WPI_VictorSPX(13);
  //@@@ JW lets start with a slower climb speed for testing  
  //int climbSpeed = 1; // value to set the climb talons to
  double climbSpeed = .5; // value to set the climb talons to
  float climbAngle = 45;  
  PowerDistributionPanel pdp = new PowerDistributionPanel();
  
  //*** PNEUMATICS ***  
  Compressor compress;  
  /*
  DoubleSolenioid solenoid1 = new DoubleSolenoid(2,5);
  DoubleSolenioid solenoid2 = new DoubleSolenoid(1,6);
  DoubleSolenioid solenoid = new DoubleSolenoid(0,7);
  */
  //@@@ JW inserting actual solonoid numbers
  //DoubleSolenoid hatchHolder = new DoubleSolenoid (0,1); // NEED A LEFT AND RIGHT
  DoubleSolenoid hatchHolder = new DoubleSolenoid (2,5); // NEED A LEFT AND RIGHT
  boolean hatchState = false;

//  DoubleSolenoid cargoHolder = new DoubleSolenoid (2,3);
  DoubleSolenoid cargoHolder = new DoubleSolenoid (1,6);
  boolean cargoState = false;

  boolean lastCameraSwitchState = false;
  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;

  int counter; // count the number of iterations in a periodic mode
  boolean climbButtonHasBeenPressed;
  //@@@ JW moved pot assignmet up a few lines.
  Potentiometer pot = new AnalogPotentiometer(0, 1, 0);
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    zeroLoopCounter();
  // @@@ JW Don't we need some configSelectedFeedbackSensor magic in here to use the encoders?
  FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  FrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  elbowTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  wristTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);

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
      camera1 = CameraServer.getInstance().startAutomaticCapture(0);
      camera2 = CameraServer.getInstance().startAutomaticCapture(1);
      server = CameraServer.getInstance().addSwitchedCamera("switched camera");
      camera1.setConnectionStrategy(ConnectionStrategy.kAutoManage); //Change connection stratagy if desired
      camera2.setConnectionStrategy(ConnectionStrategy.kAutoManage);

      //set initial states for the cylinders
      MOAC.set(DoubleSolenoid.Value.kReverse);
      hatchHolder.set(DoubleSolenoid.Value.kReverse);
      cargoHolder.set(DoubleSolenoid.Value.kReverse);      
    } 

    @Override
  public void autonomousInit() {
    zeroLoopCounter();
  }
  @Override
  public void autonomousPeriodic() {
    // @@@ JW do we need drive code here?
    incrementLoopCounter();
  }
  @Override
  public void teleopInit() {
     climbTalon.setNeutralMode(NeutralMode.Brake);
     //@@@ JW set other motorcontrollers to brake mode?
     zeroLoopCounter();
     }
  @Override
  public void teleopPeriodic() { 
    forward = leftStick.getRawAxis(1) ;
    turn = rightStick.getRawAxis(0);
    //@@@ JW test this, It might lead to jerky behavior.  And do we want pitch or roll here
    if (!((Math.abs(gyro.getRoll()) > 20) && !climbButtonHasBeenPressed))
    chassisDrive.arcadeDrive(forward, turn);    
    openHatch = leftStick.getRawButtonPressed(4);
    closeHatch = leftStick.getRawButtonPressed(3);
    camSwapButton = leftStick.getRawButton(6);      
    retractIntake = leftStick.getRawButton(5);

    toggleIntake = rightStick.getRawButton(2); //Toggle intake open / closed
    intakeButton = rightStick.getRawButton(1); //Intake cargo
    outtakeButton = rightStick.getRawButton(3); //fire cargo
    climbButton = rightStick.getRawButton(7);                                     //should this be "climbButton" or "endgameButton"

    cargoLev1 = copilotStick.getRawButton(11);
    cargoLev2 = copilotStick.getRawButton(9);
    cargoLev3 = copilotStick.getRawButton(7);
    hatchLev1 = copilotStick.getRawButton(12);
    hatchLev2 = copilotStick.getRawButton(10);
    hatchLev3 = copilotStick.getRawButton(8);
    cargoShipLev = copilotStick.getRawButton(5);
    groundLev = copilotStick.getRawButton(3);

    //Undefined buttons below 
    /** 
    rotateIntakeOutButton = copilotStick.getRawButton(8);
    rotateIntakeInButton = copilotStick.getRawButton(9);
    climbButton = leftStick.getRawButton(1);
    retractClimbCylinderButton = leftStick.getRawButton(2);
    visionTrackingButton = rightStick.getRawButton(1);            
*/

    // camera switching code
    if (camSwapButton && !lastCameraSwitchState) {
      System.out.println("Setting camera 2");
      server.setSource(camera2);
    } else if (!camSwapButton && lastCameraSwitchState) {
      System.out.println("Setting camera 1");
      server.setSource(camera1);
    }
    lastCameraSwitchState = camSwapButton;                         
    //Elevator Code
    if (groundLev) { //floor level elevator
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(0));
      setIntakeAngles(0, 0);                                   
    }
    
    //if (cargoState){
      if (cargoLev1) { //cargo rocket lvl 1
        elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(elevatorLowMidCargoRocketHeight));
        setIntakeAngles(elevatorLowMidCargoRocketHeight, lowRocketCargo);                              
      }
      if (cargoLev2) { //cargo rocket lvl 2
        elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(elevatorHighMidCargoRocketHeight));
        setIntakeAngles(elevatorHighMidCargoRocketHeight, midRocketCargo);
      }
      if (cargoLev3) { //cargo rockegt lvl 3
        elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(elevatorHighMidCargoRocketHeight));
        setIntakeAngles(elevatorHighMidCargoRocketHeight, highRocketCargo);
      }
    //}
        
    //if(hatchState){
      if (hatchLev1) { //hatch rocket lvl 1 / std hatch height
        elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(elevatorLowMidHatchRocketHeight));
        setIntakeAngles(elevatorLowMidHatchRocketHeight, lowHatch);
      }
      if (hatchLev2) { //hatch rocket lvl 2 
        elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(48)); 
        setIntakeAngles(elevatorLowMidHatchRocketHeight, midHatch);            
      }
      if (hatchLev3) {//Hatch rocket lvl 3
        elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(77));  
        setIntakeAngles(elevatorHighMidHatchRocketHeight, highHatch);            
      }
    //}

    if (cargoShipLev) { //cargo cargo ship
      elevatorTalon.set(ControlMode.MotionMagic, convertElevatorHeightToNativeUnits(43));
    } 
    // @@@ JW commented out redundant code       
    //if (outtakeButton) { //outtake
    //}
    //@@@ JW need to actually set intake roller speeds
    if (outtakeButton) { //rotate intake out
    }
    if (intakeButton) { //rotate intake in
    }
    // ***CLIMB***
    if (climbButton) { //climb
      climbButtonHasBeenPressed = true;
      elevatorTalon.set(0);
      elbowTalon.set(0);
      wristTalon.set(0);
    // if the talon stalls, as indicated by drawing too much current, put it in brake mode
    // @@@ JW I bumped the current cuttof to 40 amps
      if (pdp.getCurrent(12) < 40) { //tune this value (10)
        climbTalon.set(climbSpeed); // tune this value
      }
      else {climbTalon.set(0);}
      // if the robot is at or greater than a predetermined angle, fire the MOAC.
      if (gyro.getRoll() > climbAngle) // climbAngle is 45 degreesright now, tune
      {MOAC.set(DoubleSolenoid.Value.kForward);}
    }
    if (retractClimbCylinderButton) { //retract MOAC
      MOAC.set(DoubleSolenoid.Value.kReverse);
    }        
        
    if (visionTrackingButton) { //Vision tracking (line up the robot to the target)
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
  
  counter++;
}                      
  public int convertElevatorHeightToNativeUnits(double encValue){
    return (int) (encValue / (hubDiameter*(Math.PI)) *4096);
  }
  
  public double convertNativeUnitsToDegrees(int n) {
    return (((double) n) % 4096) / 4096 * 360;//cast n to double, then get the number of NU's away from an even rotation, 
                                              //then get the number of rotations, then conv. to degrees
  }

  public int convertDegreesToNativeUnits(double n) {
    return (int) (n / 360 * 4096); 
  }

  public void setIntakeAngles(double elevatorHeight, double goalHeight) {
    double elbowAngle = Math.acos((goalHeight - elevatorHeight)/elbowLength); //Math magic: cos = opp/adj
    elbowTalon.set(ControlMode.MotionMagic, convertDegreesToNativeUnits(elbowAngle));

    double wristAngle = -elbowAngle;
    wristTalon.set(ControlMode.MotionMagic, convertDegreesToNativeUnits(wristAngle));
  }
  
  @Override
  public void testInit() {
    motor = elevatorTalon;
  }

  @Override
  public void testPeriodic() {    
    /*motor.config_kP(slot, testP);
    motor.config_kI(slot, testI);
    motor.config_kD(slot, testD);
    motor.config_kF(slot, testF);
    */
    if (leftStick.getRawButton(1) == true) {
      motor.set(pot.get());
    } else {
      motor.set(0);
    }
   System.out.println( gyro.getPitch());
  }
  @Override
  public void disabledInit() {
    
  }
  private void zeroLoopCounter() {counter = 0;}
  private void incrementLoopCounter() {counter++;}
  
  double lastSpeed;
  /*
public double getAcceleration(WPI_TalonSRX talonToGetAccel) {

  double accel;
  double speed = (talonToGetAccel.getSelectedSensorVelocity()) * 165.1 * Math.PI  /(409600); // speed = encoder ticks per 100 ms * 1 rev / 4096 encoder ticks * 6.5 pi inches wheel circumference per revolution * 2.54 cm per 1 in * 1000 ms/1s * 1 m/100 cm
  accel = (speed - lastSpeed)/0.020;
 //if delta speed is negligible, then set acceleration to be zero
  if((speed - lastSpeed) < 1E-10)
  {accel = 0.0;}
  lastSpeed = speed;
return accel;  
}
*/


  
/**if the robot tips too far, limit its acceleration so that it can stabilise. Disable this if climbing */
  public void limitAccelerationWhenTipping() {
    if (Math.abs(gyro.getPitch()) > 20 && !climbButtonHasBeenPressed) {
      leftSide.set(leftSide.get());
      rightSide.set(rightSide.get());      
  }
    
  }
class TogglingButton {

  //inspired by Cheesy Poof's class LatchedBoolean
  //https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/lib/util/LatchedBoolean.java
  //boolean currentButtonState;
  boolean lastButtonState;
  boolean toggleState = false; 
  
  //constructors
  public TogglingButton (boolean ToggleState) {this.toggleState = ToggleState;}
  public TogglingButton () {toggleState = false;}
  
  /**
   * Update the toggled state of the button based upon input
   * @param currentButtonState the input state of the buttong
   * @return the new state of the button
   */
  public boolean update (boolean currentButtonState)  {
    //first, test whether the button was only just activated by using the test
    if (currentButtonState && !lastButtonState) {
      //If this button was only just pressed, then change the state of the toggle 
      toggleState = !toggleState; 
    }
    //last,  update the value of lastButtonState so that the toggle is not activated every 20 
    //milliseconds during the interval in which the  button is pressed
    lastButtonState = currentButtonState;
    return toggleState;
}
}
}