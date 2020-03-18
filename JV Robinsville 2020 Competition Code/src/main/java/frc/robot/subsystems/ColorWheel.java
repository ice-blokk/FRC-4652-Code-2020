/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RevColorSensor;

public class ColorWheel extends SubsystemBase {
  
  private final WPI_TalonSRX elevatorMotor;
  private final WPI_TalonSRX wheelMotor;
  private final RevColorSensor colorSensor;
  
  public final DigitalInput colorBottomLimit, colorTopLimit;
  
  public ColorWheel() {
    elevatorMotor = new WPI_TalonSRX(Constants.COLOR_ELEVATOR_PORT);
    wheelMotor = new WPI_TalonSRX(Constants.COLOR_WHEEL_PORT);
    colorSensor = new RevColorSensor();

    colorBottomLimit = new DigitalInput(3);
    colorTopLimit = new DigitalInput(2);
  }

  public void setElevatorMotor(double power){
    elevatorMotor.set(power);
  }

  public void setWheelMotor(double power){
    wheelMotor.set(power);
  }

  public void stopColorWheelMotors() {
    elevatorMotor.set(0);
    wheelMotor.set(0);
  }


  public RevColorSensor getColorSensor(){
    return colorSensor;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Color", colorSensor.color());
    SmartDashboard.putNumber("Count Yellow", colorSensor.getCount("Yellow"));
    SmartDashboard.putNumber("Count Red", colorSensor.getCount("Red"));
    SmartDashboard.putNumber("Count Green", colorSensor.getCount("Green"));
    SmartDashboard.putNumber("Count Blue", colorSensor.getCount("Blue"));
    SmartDashboard.putNumber("Red", colorSensor.getRed());
    SmartDashboard.putNumber("Green", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue", colorSensor.getBlue());
  }
}
