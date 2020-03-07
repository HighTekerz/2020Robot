/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanoidAtlas;

public class WheelLooker extends SubsystemBase {

  //                                                                  >>>>Objects/Variables<<<<

  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429),
   kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240),
   kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114),
   kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

   private String lastColor, countingColor;
   private int timesColorIsSeen; // AKA half rotations
   private boolean tallying; 
 
   private static WheelLooker wheelLooker = null;

  public static WheelLooker getInstance() {
    if (wheelLooker == null) {
      return wheelLooker = new WheelLooker();
    } else

    {
      return wheelLooker;
    }
  }

  /**
   * Creates a new WheelLooker.
   */
  private WheelLooker() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

//                                                                            >>>>Methods<<<<

public String getColor() {
  /**
* The method GetColor() returns a normalized color value from the sensor and can be
* useful if outputting the color to an RGB LED or similar. To
* read the raw color, use GetRawColor().
* 
* The color sensor works best when within a few inches from an object in
* well lit conditions (the built in LED is a big help here!). The farther
* an object is the more light from the surroundings will bleed into the 
* measurements and make it difficult to accurately determine its color.
*/
Color detectedColor = m_colorSensor.getColor();
ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
String colorString;
if (match.color == kBlueTarget) {
colorString = "B";
} else if (match.color == kRedTarget) {
colorString = "R";
} else if (match.color == kGreenTarget) {
colorString = "G";
} else if (match.color == kYellowTarget) {
colorString = "Y";
} else {
colorString = "Unknown";
}
return colorString;
}

public void startCounting() {
    countingColor = getColor();
    timesColorIsSeen = 0;
    tallying = true;
  }

  public void stopCounting(){
    tallying = false;
  }

  public String getCountingColor() {
    return countingColor;
    }
    
  public boolean counted3Spins() {
    if (timesColorIsSeen >= 6) {
      return true;
    } else {
      return false;
    }
  }

  private void tallySightings() {
    if (getColor() == countingColor && getColor() != lastColor) {
      timesColorIsSeen++;
    } else {
      lastColor = getColor();
    }
  }

  public boolean colorAligned(){
    if(DriverStation.getInstance().getGameSpecificMessage() == getColor()){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {
    if (tallying) {
      tallySightings();
    }
  }
}