/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class RevColorSensor {

    private String colorString;
    private final I2C.Port i2cPort;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;
    private final Color kBlueTarget;
    private final Color kGreenTarget;
    private final Color kRedTarget;
    private final Color kYellowTarget;
    private Color detectedColor;
    private ColorMatchResult match;

    private int count1 = 0, count2 = 0, count3 = 0, count4 = 0;
    public final static int TARGROT = 7;

    public RevColorSensor(){
        i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
        colorMatcher = new ColorMatch();
        kBlueTarget = ColorMatch.makeColor(0.231, 0.453, 0.314); //OG 0.143, 0.427, 0.429
        kGreenTarget = ColorMatch.makeColor(0.250, 0.499, 0.249); //OG 0.197, 0.561, 0.240
        kRedTarget = ColorMatch.makeColor(0.451, 0.381, 0.167); //OG 0.561, 0.232, 0.114
        kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget); 
    }

    public String color(){
        detectedColor = colorSensor.getColor();
        match = colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kBlueTarget) {
            colorString = "Blue";
          } else if (match.color == kRedTarget) {
            colorString = "Red";
          } else if (match.color == kGreenTarget) {
            colorString = "Green";
          } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
          } else {
            colorString = "Unknown";
          }
          return colorString;
    }

    public Color getDetColor(){
        detectedColor = colorSensor.getColor();
        return detectedColor;
    }

    public double getRed(){
        detectedColor = colorSensor.getColor();
        return detectedColor.red;
    }

    public double getGreen(){
        detectedColor = colorSensor.getColor();
        return detectedColor.green;
    }

    public double getBlue(){
        detectedColor = colorSensor.getColor();
        return detectedColor.blue;
    }

    public double getConfidence(){
        match = colorMatcher.matchClosestColor(detectedColor);
        return match.confidence;
    }

    public int getCount(String col) {
        switch (col){
            case "Red":
                return count1;
            case "Green":  
                return count2;
            case "Blue":
                return count3;
            case "Yellow":
                return count4;
            default:
                return 0;
       }    
    }

    public void updateCounts(String col){
        switch (col){
            case "Red":
                count1 += 1;
                break;
            case "Green":  
                count2 += 1;
                break;
            case "Blue":
                count3 += 1;
                break;
            case "Yellow":
                count4 += 1;
                break;
            default:
                break;
       }    
   }

   public void resetCounts(){
    count1 = 0;
    count2 = 0;
    count3 = 0;
    count4 = 0;
}

}
