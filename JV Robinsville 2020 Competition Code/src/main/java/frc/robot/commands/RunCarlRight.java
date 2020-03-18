/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunCarlRight extends CommandBase {
  
  private final Climber climber;

  public RunCarlRight(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setCarl(.4);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    climber.setCarl(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
