package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class Vision {
  private int bRate;
  private int counting;
  private int xVal; //In order of appearance
  private int yVal;
  private int wVal;
  private int hVal;
  private int distVal;
  private int confVal;
  private int stopDistance;
  private int confidenceThreshold;
  private int delayCount;
  private int arduinoCounter; // loop counter passed from arduino for timing checks
  private int startOfDataStream;
  private int endOfDataStream;
  private int blocksSeen;
  private SerialPort arduino;
  final int leftMax = 154;
  final int rightMax = 162;
  final int min = 0;
  public void removeFreakingAnnoyingVSCodeWarnings(){if(stopDistance + confidenceThreshold +delayCount +startOfDataStream+endOfDataStream==0){}}
  public Vision(int baud, int stop) {
    counting = 0;
    bRate = baud;
    xVal = 0;
    yVal = 0;
    wVal = 0;
    hVal = 0;
    distVal = 0;
    confVal = 0;
    blocksSeen = 0;
    arduinoCounter = 0;
    stopDistance = stop;
    confidenceThreshold = 500;
    delayCount = 1;
    arduino = startArduino();
  }

  public Vision(int baud, int stop, int delay, int conf) {
    counting = 0;
    bRate = baud;
    xVal = 0;
    yVal = 0;
    wVal = 0;
    hVal = 0;
    distVal = 0;
    confVal = 0;
    blocksSeen = 0;
    arduinoCounter = 0;
    stopDistance = stop;
    confidenceThreshold = conf;
    delayCount = delay;
    arduino = startArduino();
  }

  public Vision(int baud) {
    counting = 0;
    bRate = baud;
    xVal = 0;
    yVal = 0;
    wVal = 0;
    hVal = 0;
    distVal = 0;
    confVal = 0;
    blocksSeen = 0;
    arduinoCounter = 0;
    stopDistance = 200;
    confidenceThreshold = 500;
    delayCount = 1;
    startArduino();

  }

  public SerialPort getArduino() {
    return arduino;
  }
  public SerialPort startArduino() {
    try {
      SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kUSB);
      System.out.println("Connected to kUSB");
      return arduino;
    } catch (Exception e) {
      System.out.println("Couldn't connect to kUSB, trying kUSB1");
      try {
        SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kUSB2);
        System.out.println("Connected to kUSB2");
        return arduino;
      } catch (Exception e2) {
        System.out.println("Not connected to any of the USB ports, trying MXP spot");
        try {
          SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kMXP);
          System.out.println("Connected to MXP port");
          return arduino;
        } catch (Exception eMXP) {
          System.out.println("Not Connected to MXP port, trying Onboard");
          try {
            SerialPort arduino = new SerialPort(bRate, SerialPort.Port.kOnboard);
            System.out.println("Connected to Onboard");
            return arduino;
          } catch (Exception eOnboard) {
            System.out.println("Not connected to any ports on the RoboRIO");
            return null;
          } // catch (Exception eOnboard)
        }
      }
    }
  }
  /**
   * Primary parser: Other methods are for legacy reasons and for edge cases
   */
  public boolean parseJunk() {
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
        //System.out.println("Bad String from Arduino: Doesn't start with Block");
        return false;
      }
    } else {
      //System.out.println("Bad String from Arduino: no carriage return character or too short");
      return false;
    }
    return true;
  }
  //Getters
  public int getX() {return xVal;}
  public int getY() {return yVal;}
  public int getW() {return wVal;}
  public int getH() {return hVal;}
  public int getDist() {return distVal;}
  public int getConf() {return confVal;}
  public int getBlocksSeen() {return blocksSeen;}
  public int getArduinoCounter() {return arduinoCounter;}

  //Better getters
  public boolean isOnLeft(){return (xVal > min && xVal < leftMax && distVal > 500);}
  public boolean isInMiddle(){return (xVal >= leftMax && xVal <= rightMax && distVal > 500);}
  public boolean isOnRight(){return(xVal > rightMax && xVal < 316 && distVal > 500);}
  public String[] getArray() {
  String targetPosition = arduino.readString();
    int startOfDataStream = targetPosition.indexOf("B");
    int endOfDataStream = targetPosition.indexOf("\r");// looking for the first carriage return
    // The indexOf method returns -1 if it can't find the char in the string
    if (startOfDataStream != -1 && endOfDataStream != -1 && (endOfDataStream - startOfDataStream) > 12) {
      targetPosition = (targetPosition.substring(startOfDataStream, endOfDataStream));
      System.out.println(targetPosition);
      if (targetPosition.startsWith("Block")) {
        String[] positionNums = targetPosition.split(":");
        return positionNums;
      }
    }
    return null;
  }
  /**
   * Legacy parsers, kept in case we want to update one value without messing with the others
   * NVM, killing it with fire because that is the magic of GIT
   */
  public int parseVal(int index, int delayCount, SerialPort arduino) {
    counting = (counting + 1);
    if (counting == delayCount) {
      counting = 0;
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
    }
    if (index >= 0 && index <= 8) {
    String [] val = getArray();
    return Integer.parseInt(val[index]);
    }
    switch(index) {
      case 2 : {
        return xVal;
      }
      case 3 : {
        return yVal;
      }
      case 4 : {
        return wVal;
      }
      case 5 : {
        return hVal;
      }
      case 6 : {
        return distVal;
      }
      case 7 : {
        return confVal;
      }
      case 8 : {
        return arduinoCounter;
      }
      case 1 : {
        return blocksSeen;
      }
      default : {
        return 3000;
      }
    }
  }
}