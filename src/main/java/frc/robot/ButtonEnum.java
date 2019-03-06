package frc.robot;
public enum ButtonEnum {
    hatchToggle(0, 1), intakeOut(0, 2), 
    cameraChange(0,6), 
    elevatorUp(2,1), elevatorDown(2,2),
    elevatorLowHatch(2,7), elevatorMidHatch(2,8), elevatorHighHatch(2,9),
    elevatorLowBall(2,10), elevatorMidBall(2,11), elevatorHighBall(2,12),
    elevatorCargoShipHatch(2,6), elevatorCargoShipBall(2,7), elevatorFloor(2,9); 
    final private int buttonNum;
  final private int joystickNum;
  /**
   * If there is no boolean nor third double, it is a boring old button
   * @param numberOfJoystick the port number of the buttons joystick
   * @param numberOfButton the button number on the joystick of the button
   */
  private ButtonEnum(int numberOfJoystick, int numberOfButton) {
    buttonNum = numberOfButton; 
    joystickNum = numberOfJoystick;
  }
  public int getButtonNum() {return buttonNum;}   
  public int getJoystickNum() {return joystickNum;}  
}