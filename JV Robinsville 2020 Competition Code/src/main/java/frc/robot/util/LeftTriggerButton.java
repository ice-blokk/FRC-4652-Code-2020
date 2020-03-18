/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;



/**
 * Add your docs here.
 */
public class LeftTriggerButton extends Button{
    private final GenericHID joystick;
    private final double threshold;

    public LeftTriggerButton(GenericHID joystick, double threshold) {
        requireNonNullParam(joystick, "joystick", "LeftTriggerButton");
        this.joystick = joystick;
        this.threshold = threshold;
    }

    @Override
    public boolean get() {
        return joystick.getRawAxis(2) > threshold;
    }

}
