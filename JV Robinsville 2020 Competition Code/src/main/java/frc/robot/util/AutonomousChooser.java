/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autocommands.AutoDriveForwardDistance;
import frc.robot.commands.autocommands.AutoStrafeDistance;
import frc.robot.subsystems.Box;
import frc.robot.subsystems.DriveTrain;

/**
 * This is the AutonomousChooser. Auto Cases are picked from the SmartDashboard
 * then scheduled through Robot.java (in autonomousInit()).
 * This class handles the creation of the Auto Chooser and Auto Commands
 */
public class AutonomousChooser {

    /** Some Field and Robot Specs/Notes
     * 
     * The Robot is ~2 2/3 ft long (~32)
     * 
     * The initiation line (starting line) is 10 feet fromt the alliance wall
     * 
     * From the center of the target to the (right) alliance wall is 7.8883 ft (94.66 in)
     * 
     * From the center of the arena to the left side of the target is 3.64583 ft. (43.75 in.)
     * 
     * Alliance stations 1 and 3 (the ones that are curved) protrude inwards about 2 1/2 ft. (30 in.)
     *  This measurement is the size of the proteceted triangle of the target and feeder station. The
     *  triangle protrudes into the field a little more than the player station does.
     * 
     * All of the starting positions of the robot will be the back side of the bumper over
     *  the initiation line.
     * 
     * Unless otherwise noted, the Robot will stay with the front (box side) facing the alliance station wall
     * 
     * All "left" and "right" auto cases will be 1 foot from their respective arena wall
     *  i.e. in the auto case "RightScoreLowGoal", the Robot will start with the back side of the bumper
     *  on the initiation line, front facing the alliance station wall, and 1 foot from the right arena wall
     * 
     * Middle Auto Cases start at the middle most line at the center of the field (check field diagrams)
     *  I believe the center line is a dark gray line in the middle of the carpet. It is not visible in the 
     *  regular field art, but is visible on the actual field and field diagrams.
     */

    private final DriveTrain drivetrain;
    private final Box box;
    private final double autoDelay;

    public final SequentialCommandGroup 
        None, 
        TestMoveAndStrafe, 
        InitiationLine,
        StraightScoreLowGoal, 
        RightScoreLowGoal, 
        MiddleScoreLowGoal, 
        LeftScoreLowGoal,
        DelayStraightScoreLowGoal,
        DelayRightScoreLowGoal,
        DelayMiddleScoreLowGoal,
        DelayLeftScoreLowGoal;

    private final static SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();
    public static SequentialCommandGroup choice = chooser.getSelected();

    public AutonomousChooser(DriveTrain drivetrain, Box box, double autoDelay) {
        this.drivetrain = drivetrain;
        this.box = box;
        this.autoDelay = autoDelay;

        /** AUTONOMOUS COMMANDS
         * 
         * Tested Cases: none so far ahhhhh
         */
        None = null;

        TestMoveAndStrafe = new SequentialCommandGroup(
            new AutoDriveForwardDistance(drivetrain, .5, 5.0),
            new AutoStrafeDistance(drivetrain, .5, 5.0)
        );

        InitiationLine = new SequentialCommandGroup(
            new AutoDriveForwardDistance(drivetrain, .5, 3)
        );

        StraightScoreLowGoal = new SequentialCommandGroup(
            new AutoDriveForwardDistance(drivetrain, .5, 7.5),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.5, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 7.5)
        );

        RightScoreLowGoal = new SequentialCommandGroup(
            new AutoDriveForwardDistance(drivetrain, .5, 5),
            new AutoStrafeDistance(drivetrain, -.4, 6.8),
            new AutoDriveForwardDistance(drivetrain, .5, 2),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.4, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 6.8)
        );

        MiddleScoreLowGoal = new SequentialCommandGroup(
            new AutoDriveForwardDistance(drivetrain, .5, 7.5),
            new AutoStrafeDistance(drivetrain, .5, 4),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.5, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 6.8)
        );

        // The first strafe to the right is an approximation
        LeftScoreLowGoal = new SequentialCommandGroup(
            new AutoDriveForwardDistance(drivetrain, .5, 5),
            new AutoStrafeDistance(drivetrain, .5, 16),
            new AutoDriveForwardDistance(drivetrain, .5, 2),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.4, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 6.8)
        );

        DelayStraightScoreLowGoal = new SequentialCommandGroup(
            new WaitCommand(autoDelay),
            new AutoDriveForwardDistance(drivetrain, .5, 7.5),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.5, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 7.5)
        );

        DelayRightScoreLowGoal = new SequentialCommandGroup(
            new WaitCommand(autoDelay),
            new AutoDriveForwardDistance(drivetrain, .5, 5),
            new AutoStrafeDistance(drivetrain, -.4, 6.8),
            new AutoDriveForwardDistance(drivetrain, .5, 2),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.4, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 6.8)
        );

        DelayMiddleScoreLowGoal = new SequentialCommandGroup(
            new WaitCommand(autoDelay),
            new AutoDriveForwardDistance(drivetrain, .5, 7.5),
            new AutoStrafeDistance(drivetrain, .5, 4),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.5, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 6.8)
        );

        DelayLeftScoreLowGoal = new SequentialCommandGroup(
            new WaitCommand(autoDelay),
            new AutoDriveForwardDistance(drivetrain, .5, 5),
            new AutoStrafeDistance(drivetrain, .5, 16),
            new AutoDriveForwardDistance(drivetrain, .5, 2),
            new InstantCommand(() -> box.down(), box),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> box.resetAllServos(), box),
                new AutoDriveForwardDistance(drivetrain, -.4, 2.5)),
            new AutoStrafeDistance(drivetrain, .5, 6.8)
        );

        // Autonomous Chooser
        chooser.setDefaultOption("None", None);
        chooser.addOption("Test Move And Strafe", TestMoveAndStrafe);
        chooser.addOption("Initiation Line", InitiationLine);
        chooser.addOption("Straight Score Low Goal", StraightScoreLowGoal);
        chooser.addOption("Right Score Low Goal", RightScoreLowGoal);
        chooser.addOption("Middle Score Low Goal", MiddleScoreLowGoal);
        chooser.addOption("Left Score Low Goal", LeftScoreLowGoal);
        chooser.addOption("Delay Straight Score Low Goal", DelayStraightScoreLowGoal);
        chooser.addOption("Delay Right Score Low Goal", DelayRightScoreLowGoal);
        chooser.addOption("Delay Middle Score Low Goal", DelayMiddleScoreLowGoal);
        chooser.addOption("Delay Left Score Low Goal", DelayLeftScoreLowGoal);

        // Puts Autonomous Chooser on SmartDashboard
        SmartDashboard.putData("Autonomous Choices", chooser);

    }

    public Command getSelectedAutoCommand() {
        return choice;
    }

    public double getInputAutoDelay() {
        return autoDelay;
    }

}
