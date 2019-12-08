package parkingRobot;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl.ControlMode;

/**
 * interface for the main module 'Control', providing methods for executing the algorithms to
 * control the robot according to different modes which is set by guidance.
 * 
 * @author RST
 */
public interface IControl {
	// Inputs
	
	
	/**
	 * The predefined control modes
	 */
	public enum ControlMode {
		

		
		/**
		 * folgt schwarzer Linie
		 */
		FAST,
		
		SLOW,
		
		/**
		 * einparken
		 */
		PARK_CTRL,
		
		/**
		 * v/w-Control
		 */
		VW_CTRL,
		
		/**
		 * eine Zielpsoe anfahren
		 */
		SETPOSE,
		
		/**
		 * NXT in Ruhe versetzen
		 */
		INACTIVE,
		/**
		 * Pfadfolger
		 */
		PFADFOLGER;
	}
	
		
	/**
	 * set the required speed
	 * 
	 * @param velocity the velocity of the robot to be set
	 */	
	public void setVelocity(double velocity);

	
	/**
	 *  set the required angular velocity
	 *  
	 * @param angularVelocity  the angular velocity to be set
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
	

	
	public void setDriveFor(double x, double y, double phi, double v, double omega, Pose startPose);
	
	/**
	 * set the current control mode
	 * 
	 * @param ctrl_mode parameter for control mode which is defined by Guidance 
	 */
	public void setCtrlMode(ControlMode ctrl_mode);
	
	public ControlMode getCtrlMode();
	
	/**
	 * set start time
	 * @param startTime start time
	 */
	public void setStartTime(int startTime);
	
	
	/**
	 * execute the selected algorithms for control which was set by guidance
	 */
	public void exec_CTRL_ALGO();

}

