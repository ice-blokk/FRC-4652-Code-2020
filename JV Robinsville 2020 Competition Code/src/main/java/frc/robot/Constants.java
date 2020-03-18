package frc.robot;

//Every constant the robot uses, in one package.
//You can nest classes here to organize the data better, if you want.
//Make sure you name the constants so that you know what they are by looking at them
public class Constants {

	//CAN IDs
	public static final int FL_DRIVE_PORT = 1,
							FL_TURN_PORT = 5,
							FR_DRIVE_PORT = 2,
			    			FR_TURN_PORT = 6,
			   				BL_DRIVE_PORT = 3,
			  				BL_TURN_PORT = 7,
			  				BR_DRIVE_PORT = 4,
							BR_TURN_PORT = 8;
							 
	public static final int CLIMB_MOTOR_PORT = 9,
							WINCH_MOTOR_PORT = 10,
							CRAWL_MOTOR_PORT = 14;
	
	public static final int COLOR_ELEVATOR_PORT = 11,
							COLOR_WHEEL_PORT = 12;
	
	public static final int SHOOT_MOTOR_PORT = 13;

	public static final int TILT_RIGHT_PORT = 0,
							TILT_LEFT_PORT = 1,
							DUMP_RIGHT_PORT = 2,
							DUMP_LEFT_PORT = 3,
							SHOOT_GATE_PORT = 4,
							SHOOT_TILT_PORT = 5;

	public static final int FL_OFFSET = 0,
			  				FR_OFFSET = 0,
							BL_OFFSET = 0,
			  				BR_OFFSET = 0;

	public static final int DRIVER_PORT = 0,
							OPERATOR_PORT= 1,
							SRX_TIMEOUT_MS = 10,
							SRX_PIDLOOPIDX = 0;
	
	public static final double SWERVE_P_GAIN = .945,
							   SWERVE_I_GAIN = 0,
							   SWERVE_D_GAIN = 49.5;

	//Some physical dimensions of the robot for swerve calculations
	public static final double WHEELBASE_INCHES = 23.75,
							   TRACKWIDTH_INCHES = 21.5,
							   TURN_RADIUS_INCHES = Math.hypot(WHEELBASE_INCHES, TRACKWIDTH_INCHES),
							   MODULE_FULL_ROTATION = 4096 / 1.2;

	//This basic function can be used to filter noise out of a joystick input
	public static double filter(final double a) {
		if(Math.abs(a) < .08) {
			return 0;
		}
		return a;
	}

	// Auto Constants
	public static final double MOD_FR_TEN_FEET_TRAVELED = 60,
							   MOD_FL_TEN_FEET_TRAVELED = 60,
							   MOD_BR_TEN_FEET_TRAVELED = 60,
							   MOD_BL_TEN_FEED_TRAVELED = 60;

	public static final double MOD_FR_FOOT_TRAVELED = MOD_FR_TEN_FEET_TRAVELED / 10,
							   MOD_FL_FOOT_TRAVELED = MOD_FL_TEN_FEET_TRAVELED / 10,
							   MOD_BR_FOOT_TRAVELED = MOD_BR_TEN_FEET_TRAVELED / 10,
							   MOD_BL_FOOT_TRAVELED = MOD_BL_TEN_FEED_TRAVELED / 10;

	public static final double AVERAGE_FOOT = (MOD_FR_FOOT_TRAVELED + MOD_FL_FOOT_TRAVELED + 
		MOD_BR_FOOT_TRAVELED + MOD_BL_FOOT_TRAVELED) / 4;
	
	
}