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

public class Robot extends TimedRobot {

  WPI_TalonSRX motor;
  WPI_TalonSRX elevatorTalon = new WPI_TalonSRX(9);
  WPI_TalonSRX elbowTalon = new WPI_TalonSRX(10);
  WPI_TalonSRX wristTalon = new WPI_TalonSRX(7);
  //@@@ JW need intake victor assignments
  WPI_VictorSPX leftIntakeRoller = new WPI_VictorSPX(22);
  WPI_VictorSPX rightIntakeRoller = new WPI_VictorSPX(23);

Joystick leftStick = new Joystick(0);
  Joystick rightStick = new Joystick(1);
  Joystick copilotStick = new Joystick(2);
  double forward;
  double turn; 
  // *** Buttons ***
  boolean overrideButton;
  TogglingButton hatchToggle;
  boolean climbRetractCyl;
  boolean climbRetractArm;    
  boolean cylButton;
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
  boolean hatch;
  boolean retractClimbCylinderButton;
  boolean visionTrackingButton;
  boolean camSwapButton;
  boolean retractIntake;
  TogglingButton toggleIntake;
  boolean intakeBool;

  DoubleSolenoid MOAC = new DoubleSolenoid (0,7); // mother of all cylinders

  WPI_TalonSRX climbTalon = new WPI_TalonSRX(12);
  WPI_VictorSPX climbFollower = new WPI_VictorSPX(13);

  double climbSpeed = .5; // value to set the climb talons to

  Compressor compress;  

  DoubleSolenoid hatchHolder = new DoubleSolenoid (2,5); // NEED A LEFT AND RIGHT

  DoubleSolenoid cargoHolder = new DoubleSolenoid (1,6);

  UsbCamera camera1;
  UsbCamera camera2;

  public void robotInit() {

  FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  FrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  elbowTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  wristTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  
  compress = new Compressor(0);
  compress.setClosedLoopControl(true);

  camera1 = CameraServer.getInstance().startAutomaticCapture(0);
  camera2 = CameraServer.getInstance().startAutomaticCapture(1);

  MOAC.set(DoubleSolenoid.Value.kReverse);
  hatchHolder.set(DoubleSolenoid.Value.kReverse);
  cargoHolder.set(DoubleSolenoid.Value.kReverse);   
  }
 public void autonomousInit() {
    climbTalon.setNeutralMode(NeutralMode.Brake);
  }
@Override
  public void autonomousPeriodic() {
    forward = leftStick.getRawAxis(1) ;
    turn = rightStick.getRawAxis(0);
    //@@@ JW test this, It might lead to jerky behavior.  And do we want pitch or roll here
   // if (!((Math.abs(gyro.getRoll()) > 20) && !climbButtonHasBeenPressed))
  /* if (((Math.abs(gyro.getRoll()) < 20) && !climbButtonHasBeenPressed) || overrideButton) {
    chassisDrive.arcadeDrive(forward, turn); } */
    chassisDrive.arcadeDrive(forward, turn);
    overrideButton = leftStick.getRawButton(1);
    hatch = leftStick.getRawButtonPressed(4);
    camSwapButton = leftStick.getRawButton(6);      
    retractIntake = leftStick.getRawButton(5);
    cylButton = leftStick.getRawButton(7);
    climbButton = leftStick.getRawButton(8);
    climbRetractCyl = leftStick.getRawButton(9);
    climbRetractArm = leftStick.getRawButton(10);
    intakeBool = rightStick.getRawButton(2); //Toggle intake open / closed
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
      if (cylButton) // climbAngle is 45 degreesright now, tune
      {MOAC.set(DoubleSolenoid.Value.kForward);}
    }
    if (retractClimbCylinderButton) { //retract MOAC
      MOAC.set(DoubleSolenoid.Value.kReverse);
    }        
        
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
   // if (!((Math.abs(gyro.getRoll()) > 20) && !climbButtonHasBeenPressed))
   //if (((Math.abs(gyro.getRoll()) < 20) && !climbButtonHasBeenPressed) || overrideButton) {
    chassisDrive.arcadeDrive(forward, turn);// }
    overrideButton = leftStick.getRawButton(1);
    hatch = leftStick.getRawButtonPressed(4);
    camSwapButton = leftStick.getRawButton(6);      
    retractIntake = leftStick.getRawButton(5);
    cylButton = leftStick.getRawButton(7);
    intakeBool = rightStick.getRawButton(2); //Toggle intake open / closed
    intakeButton = rightStick.getRawButton(1); //Intake cargo
    outtakeButton = rightStick.getRawButton(3); //fire cargo
    climbButton = leftStick.getRawButton(8);
    climbRetractCyl = leftStick.getRawButton(9);
    climbRetractArm = leftStick.getRawButton(10);                                      //should this be "climbButton" or "endgameButton"

    cargoLev1 = copilotStick.getRawButton(11);
    cargoLev2 = copilotStick.getRawButton(9);
    cargoLev3 = copilotStick.getRawButton(7);
    hatchLev1 = copilotStick.getRawButton(12);
    hatchLev2 = copilotStick.getRawButton(10);
    hatchLev3 = copilotStick.getRawButton(8);
    cargoShipLev = copilotStick.getRawButton(5);
    groundLev = copilotStick.getRawButton(3);

    retractClimbCylinderButton = leftStick.getRawButton(8);
 
     if (pdp.getCurrent(12) < 40) { //tune this value (10)
        climbTalon.set(climbSpeed); // tune this value
      }
      else {climbTalon.set(0);}
      // if the robot is at or greater than a predetermined angle, fire the MOAC.
      if (cylButton) // climbAngle is 45 degreesright now, tune
      {MOAC.set(DoubleSolenoid.Value.kForward);}
    }
    if (retractClimbCylinderButton) { //retract MOAC
      MOAC.set(DoubleSolenoid.Value.kReverse);
    }        
 