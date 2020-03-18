package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

//DO NOT TOUCH this class unless you know what you're doing.
public final class Main {
	
	private Main() {
	}

	public static void main(String... args) {
		RobotBase.startRobot(Robot::new);
	}

}