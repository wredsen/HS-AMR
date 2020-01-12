package parkingRobot;

import lejos.robotics.navigation.Pose;

/**
 * interface for the main module 'Control', providing methods for executing the algorithms to
 * control the robot according to different modes which is set by guidance.
 * 
 * @author Konstantin Wrede
 */
public interface IControl {
	
	/**
	 * control modes
	 */
	public enum ControlMode {
		

		
		/**
		 * linefollow mode at highspeed with low d-control 
		 */
		FAST,
		
		/**
		 * linefollow mode at lowspeed with high d-control 
		 */
		SLOW,
		
		/**
		 * executing parking mode
		 */
		PARK_CTRL,
		
		/**
		 * v/w-control for robot speed
		 */
		VW_CTRL,
		
		/**
		 * executing driving to position by translating and rotating
		 */
		SETPOSE,
		
		/**
		 * Sleeping mode
		 */
		INACTIVE
	}
	
		
	/**
	 * set the required speed
	 * 
	 * @param velocity setting the translatory velocity for all control sequences
	 */	
	public void setVelocity(double velocity);

	
	/**
	 *  set the required angular velocity
	 *  
	 * @param angularVelocity setting the angular velocity for all control sequences
	 */
	public void setAngularVelocity(double angularVelocity);

	/**
	 * set the destination to be driven to
	 * 
	 * @param heading the heading angle of the robot at the destination
	 * @param x the destination position in x axis
	 * @param y the destination position in y axis
	 */
	public void setDestination(double heading, double x, double y);
		
	
	/**
	 * the Robot's current position 
	 * 
	 * @param currentPosition the current position of the robot at each sampling  
	 */	
	public void setPose(Pose currentPosition); 	
	

	/**
	 * set RELATIVE destination and velocities before entering setpose-mode
	 * 
	 * @param x the destination position on x-axis in m
	 * @param y the destination position on y-axis in m 
	 * @param phi the heading angle of the robot at the destination in rad
	 * @param v translatory velocity in cm/s
	 * @param w angular velocity in rad/s
	 * @param startPose latest measured pose to start trajectory 
	 */
	public void setDriveFor(double x, double y, double phi, double v, double w, Pose startPose);
	
	
	/**
	 * set destination and calculate trajectory coefficients before parking-mode
	 * 
	 * @param startPose start pose of the trajectory
	 * @param endPose finishing pose of the trajectory 
	 */
	public void setParkingFor(Pose startPose, Pose endPose);
	
	
	/**
	 * set the current control mode
	 * 
	 * @param ctrl_mode parameter for control mode which is defined by Guidance 
	 */
	public void setCtrlMode(ControlMode ctrl_mode);
	
	
	/**
	 * get the current control mode
	 * 
	 * @return current ctrl_mode
	 */
	public ControlMode getCtrlMode();
	
	/**
	 * set start time
	 * @param startTime start time
	 */
	public void setStartTime(int startTime);
	
	
	/**
	 * execute the selected control algorithm which was set by guidance
	 */
	public void exec_CTRL_ALGO();

}

