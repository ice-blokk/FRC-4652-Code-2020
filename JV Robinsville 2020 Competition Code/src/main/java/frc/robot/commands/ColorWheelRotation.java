/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;
import frc.robot.util.RevColorSensor;

public class ColorWheelRotation extends CommandBase {

  private final ColorWheel colorWheel;
  private final RevColorSensor color;

  private int latch;
  private final String OGCOL;

  public ColorWheelRotation(ColorWheel colorWheel) {
    this.colorWheel = colorWheel;
    addRequirements(colorWheel);
    
    color = colorWheel.getColorSensor();
    OGCOL = color.color();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    latch = 0;
    color.resetCounts();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(latch == 0){
      color.updateCounts(OGCOL);
      latch++;
    }

    colorWheel.setWheelMotor(-.7);//start thingy


    if(latch == 1){
      if(OGCOL == "Red"){
        if((color.color() == "Green") && (color.getCount("Green") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Blue") && (color.getCount("Blue") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Yellow") && (color.getCount("Yellow") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == OGCOL) && (color.getCount("Green") == color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if(color.getCount(OGCOL) >= RevColorSensor.TARGROT)
          latch = 2;
      }
      else if(OGCOL == "Green"){
        if((color.color() == "Blue") && (color.getCount("Blue") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Yellow") && (color.getCount("Yellow") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Red") && (color.getCount("Red") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == OGCOL) && (color.getCount("Blue") == color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if(color.getCount(OGCOL) >= RevColorSensor.TARGROT)
          latch = 2;
      }
      else if(OGCOL == "Blue"){
        if((color.color() == "Green") && (color.getCount("Green") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Yellow") && (color.getCount("Yellow") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Red") && (color.getCount("Red") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == OGCOL) && (color.getCount("Yellow") == color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if(color.getCount(OGCOL) >= RevColorSensor.TARGROT)
          latch = 2;
      }
      else if(OGCOL == "Yellow"){
        if((color.color() == "Blue") && (color.getCount("Blue") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Green") && (color.getCount("Green") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == "Red") && (color.getCount("Red") < color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if((color.color() == OGCOL) && (color.getCount("Red") == color.getCount(OGCOL)) && (color.getCount(OGCOL) < RevColorSensor.TARGROT))
          color.updateCounts(color.color());
        else if(color.getCount(OGCOL) >= RevColorSensor.TARGROT)
          latch = 2;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorWheel.setWheelMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return latch == 2;
  }
}
