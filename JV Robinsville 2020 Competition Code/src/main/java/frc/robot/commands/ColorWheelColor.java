/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;
import frc.robot.util.RevColorSensor;

public class ColorWheelColor extends CommandBase {
  
  private final ColorWheel colorwheel;
  String gameData;
  boolean atColor;
  RevColorSensor color;


  public ColorWheelColor(ColorWheel colorwheel) {
    this.colorwheel = colorwheel;
    addRequirements(colorwheel);

    gameData = DriverStation.getInstance().getGameSpecificMessage();
    color = colorwheel.getColorSensor();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atColor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    colorwheel.setWheelMotor(-.3);

    if(gameData.length() > 0) {
      switch (gameData.charAt(0))
      {
        case 'B' :
          System.out.println("[GAME DATA]: B");
          if(color.color() == "Red") {
            atColor = true;
          }
          break;
        case 'G' :
          System.out.println("[GAME DATA]: G");
          if(color.color() == "Yellow") {
            atColor = true;
          }
          break;
        case 'R' :
          System.out.println("[GAME DATA]: R");
          if(color.color() == "Blue") {
            atColor = true;
          }
          break;
        case 'Y' :
          System.out.println("[GAME DATA]: Y");
          if(color.color() == "Green") {
            atColor = true;
          }
          break;
        default :
          System.out.println("[INFO]: Corrupt Game Data!");
          break;
      }
    } 
    else {
      System.out.println("[INFO]: No Game Data!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorwheel.setWheelMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atColor;
  }
}
