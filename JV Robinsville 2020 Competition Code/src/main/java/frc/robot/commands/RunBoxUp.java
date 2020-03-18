/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Box;

public class RunBoxUp extends CommandBase {
  
  private final Box box;

  public RunBoxUp(Box box) {
    this.box = box;
  }

  @Override
  public void execute() {
    box.up();
  }

  @Override
  public void end(boolean interrupted) {
    box.resetAllServos();
  }


  
  @Override
  public boolean isFinished() {
    return false;
  }
}
