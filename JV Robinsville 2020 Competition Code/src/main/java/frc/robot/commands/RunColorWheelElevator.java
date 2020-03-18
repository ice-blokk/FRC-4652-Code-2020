/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;

public class RunColorWheelElevator extends CommandBase {
  
  ColorWheel colorwheel;
  DoubleSupplier power;

  public RunColorWheelElevator(ColorWheel colorwheel, DoubleSupplier power) {
    this.colorwheel = colorwheel;
    this.power = power;
    addRequirements(colorwheel);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    if(power.getAsDouble() < 0) {
      if(colorwheel.colorBottomLimit.get()) {
        colorwheel.setElevatorMotor(power.getAsDouble());
      }
      else{
        colorwheel.setElevatorMotor(0);
      }
    }
    else if(power.getAsDouble() > 0) {
      if(colorwheel.colorTopLimit.get()) {
        colorwheel.setElevatorMotor(power.getAsDouble());
      }
      else {
        colorwheel.setElevatorMotor(0);
      }
    }

  }

  
  @Override
  public void end(boolean interrupted) {
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
