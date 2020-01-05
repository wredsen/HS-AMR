package parkingRobot.hsamr1;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.hsamr1.ControlRST_Ver2;
import parkingRobot.hsamr1.HmiPLT_Ver2;
import parkingRobot.hsamr1.NavigationAT_Ver2;
import parkingRobot.hsamr1.PerceptionPMP_Ver2;
import parkingRobot.hsamr1.CTR_D3.CurrentStatus;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;


/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
 * parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
 * structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
 * state machines (FSM). It may be advantageous to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be realized in one main module class.
 * Every class (except guidance) has additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module class thread is only handled in a
 * synchronized context to avoid inconsistent or corrupt data!
 */
public class CTR_D3 {
	
	static int presentationMode = 1;	// 0: Parking, 1: Move and Turn
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		DRIVING0,
		
		DRIVING1,
		
		
		DRIVING2,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
		
		
		TURN_CW,
		
		
		TURN_CCW,
		
		
		TURN_CCW0,
		
		
		LINE_FOLLOW_FAST,
		
		
		LINE_FOLLOW_SLOW,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}
	
	
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;
	
	
	/**
	 * one line of the map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * This documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(  0,  0, 180,  0);
	static Line line1 = new Line(180,  0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30,  30, 30);
	static Line line5 = new Line( 30, 30,  30, 60);
	static Line line6 = new Line( 30, 60,   0, 60);
	static Line line7 = new Line(  0, 60,   0,  0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};
	
	
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {		
        currentStatus = CurrentStatus.INACTIVE;
        lastStatus    = CurrentStatus.EXIT;
    
		
		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);
		
		IMonitor monitor = new Monitor_Ver2();
		
		IPerception perception = new PerceptionPMP_Ver2(leftMotor, rightMotor, monitor);
		//perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT_Ver2(perception, monitor);
		IControl    control    = new ControlRST_Ver2(perception, navigation, leftMotor, rightMotor, monitor);
		//INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);
		
		monitor.startLogging();
		
		if (presentationMode == 0) {
		
		while(true) {
			control.showCTRData();
			
        	switch ( currentStatus )
        	{
        		
				case DRIVING1:
					
					
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING1 ){
						Pose startPose = navigation.getPose();
						startPose.setHeading(0);
						Pose endPose = new Pose(.60f,-.25f,0);
						//control.setDriveFor(0, 0, 0, 10, 0, navigation.getPose());
						control.setParkingData(startPose, endPose);
						control.setCtrlMode(ControlMode.PARK_CTRL);
					}
					
					
					//While action				
						
					//showData_linesensor(perception);
					
					//State transition check
					currentStatus = CurrentStatus.DRIVING1;
				    lastStatus = currentStatus;
				    if (control.getCtrlMode() == ControlMode.INACTIVE) {
				    	Button.ENTER.waitForPressAndRelease();
				    	currentStatus = CurrentStatus.DRIVING2;
				    	Thread.sleep(500);
				    }
					
				    
					if ( Button.ENTER.isDown() ){
	  	        		currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
				    
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING1 ){
						//nothing to do here
					}
					break;	
				
				case TURN_CCW:
					//Into action
					if ( lastStatus != CurrentStatus.TURN_CCW ){
						control.setDriveFor(0, 0, Math.toRadians(90), 0, Math.toRadians(60), navigation.getPose()); // 90deg @ 15deg/s
						control.setCtrlMode(ControlMode.SETPOSE);
					}
					
					//State transition check
					currentStatus = CurrentStatus.TURN_CCW;
				    lastStatus = currentStatus;
				    if ((control.getCtrlMode() == ControlMode.INACTIVE)) {
				    	currentStatus = CurrentStatus.DRIVING2;
				    	Thread.sleep(500);
				    }
				    
					
					break;	
					
				case DRIVING2:
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING2 ){
						control.setDriveFor(-0.12, 0, 0, -10, 0, navigation.getPose());	// 0,3m @ 5cm/s
						control.setCtrlMode(ControlMode.SETPOSE);
						//control.setDriveFor(0, 0, Math.toRadians(120), 0, Math.toRadians(35), navigation.getPose()); // 90deg @ 15deg/s
						//control.setDriveFor(0, 0, Math.toRadians(-120), 0, Math.toRadians(-50), navigation.getPose()); // -90deg @ -30deg/s
					}
					
					
					//While action				
						
					//showData_linesensor(perception);
					
					//State transition check
					currentStatus = CurrentStatus.DRIVING2;
				    lastStatus = currentStatus;
				    if (control.getCtrlMode() == ControlMode.INACTIVE) {
				    	currentStatus = CurrentStatus.TURN_CW;
				    	Thread.sleep(500);
				    }
					
				    
					if ( Button.ENTER.isDown() ){
	  	        		currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
				    
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING2 ){
						//nothing to do here
					}
					break;	
					
				case TURN_CW:
					//Into action
					if ( lastStatus != CurrentStatus.TURN_CW ){
						control.setDriveFor(0, 0, Math.toRadians(-90), 0, Math.toRadians(-90), navigation.getPose()); // -90deg @ -30deg/s
						control.setCtrlMode(ControlMode.SETPOSE);
					}
					
					//State transition check
					currentStatus = CurrentStatus.TURN_CW;
				    lastStatus = currentStatus;
				    if (control.getCtrlMode() == ControlMode.INACTIVE) {
				    	currentStatus = CurrentStatus.INACTIVE;
				    }
						
					break;
					
				case INACTIVE:
					
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
						LCD.drawString("Pause!", 0, 0);
					}
					
					//While action
					{
						//nothing to do here
					}
					
					
					//State transition check
					lastStatus = currentStatus;
							
					if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.DRIVING1;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.INACTIVE ){
						//nothing to do here
					}
									
					break;
				case EXIT:
				
					/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
					// monitor.sendOfflineLog();
					*/
					monitor.stopLogging();
					System.exit(0);
					break;
			default:
				break;
        	}
        		
        	Thread.sleep(100);        	
		}
		}
	
	if (presentationMode == 1) {
		while(true) {
			control.showCTRData();
		
        	switch ( currentStatus )
        	{
	        	case DRIVING0:
					
					
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING0 ){
						control.setDriveFor(1.0, 0, 0, 10, 0, navigation.getPose());	// 1,2m @ 10cm/s
						control.setCtrlMode(ControlMode.SETPOSE);
						//control.setDriveFor(0, 0, Math.toRadians(120), 0, Math.toRadians(35), navigation.getPose()); // 90deg @ 15deg/s
						//control.setDriveFor(0, 0, Math.toRadians(-120), 0, Math.toRadians(-50), navigation.getPose()); // -90deg @ -30deg/s
					}
					
					
					//While action				
						
					//showData_linesensor(perception);
					
					//State transition check
					currentStatus = CurrentStatus.DRIVING0;
				    lastStatus = currentStatus;
				    if (control.getCtrlMode() == ControlMode.INACTIVE) {
				    	currentStatus = CurrentStatus.TURN_CCW0;
				    	Thread.sleep(500);
				    }
					
				    
					if ( Button.ENTER.isDown() ){
	  	        		currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
				    
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING0 ){
						//nothing to do here
					}
					break;
					
        		case TURN_CCW0:
					//Into action
					if ( lastStatus != CurrentStatus.TURN_CCW0 ){
						control.setDriveFor(0, 0, Math.toRadians(200), 0, Math.toRadians(60), navigation.getPose()); // 90deg @ 15deg/s
						control.setCtrlMode(ControlMode.SETPOSE);
					}
					
					//State transition check
					currentStatus = CurrentStatus.TURN_CCW0;
				    lastStatus = currentStatus;
				    if ((control.getCtrlMode() == ControlMode.INACTIVE)) {
				    	currentStatus = CurrentStatus.DRIVING1;
				    	Thread.sleep(500);
				    }
				break;
        		
				case DRIVING1:
					
					
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING1 ){
						control.setDriveFor(-0.30, 0, 0, 10, 0, navigation.getPose());	// 1,2m @ 10cm/s
						control.setCtrlMode(ControlMode.SETPOSE);
						//control.setDriveFor(0, 0, Math.toRadians(120), 0, Math.toRadians(35), navigation.getPose()); // 90deg @ 15deg/s
						//control.setDriveFor(0, 0, Math.toRadians(-120), 0, Math.toRadians(-50), navigation.getPose()); // -90deg @ -30deg/s
					}
					
					
					//While action				
						
					//showData_linesensor(perception);
					
					//State transition check
					currentStatus = CurrentStatus.DRIVING1;
				    lastStatus = currentStatus;
				    if (control.getCtrlMode() == ControlMode.INACTIVE) {
				    	currentStatus = CurrentStatus.TURN_CCW;
				    	Thread.sleep(500);
				    }
					
				    
					if ( Button.ENTER.isDown() ){
	  	        		currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
				    
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING1 ){
						//nothing to do here
					}
					break;	
				
				case TURN_CCW:
					//Into action
					if ( lastStatus != CurrentStatus.TURN_CCW ){
						control.setDriveFor(0, 0, Math.toRadians(0), 0, Math.toRadians(60), navigation.getPose()); // 90deg @ 15deg/s
						control.setCtrlMode(ControlMode.SETPOSE);
					}
					
					//State transition check
					currentStatus = CurrentStatus.TURN_CCW;
				    lastStatus = currentStatus;
				    if ((control.getCtrlMode() == ControlMode.INACTIVE)) {
				    	currentStatus = CurrentStatus.DRIVING2;
				    	Thread.sleep(500);
				    }
				    
					
					break;	
					
				case DRIVING2:
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING2 ){
						control.setDriveFor(0.12, 0, 0, -10, 0, navigation.getPose());	// 0,3m @ 5cm/s
						control.setCtrlMode(ControlMode.SETPOSE);
						//control.setDriveFor(0, 0, Math.toRadians(120), 0, Math.toRadians(35), navigation.getPose()); // 90deg @ 15deg/s
						//control.setDriveFor(0, 0, Math.toRadians(-120), 0, Math.toRadians(-50), navigation.getPose()); // -90deg @ -30deg/s
					}
					
					
					//While action				
						
					//showData_linesensor(perception);
					
					//State transition check
					currentStatus = CurrentStatus.DRIVING2;
				    lastStatus = currentStatus;
				    if (control.getCtrlMode() == ControlMode.INACTIVE) {
				    	currentStatus = CurrentStatus.TURN_CW;
				    	Thread.sleep(500);
				    }
					
				    
					if ( Button.ENTER.isDown() ){
	  	        		currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
				    
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING2 ){
						//nothing to do here
					}
					break;	
					
				case TURN_CW:
					//Into action
					if ( lastStatus != CurrentStatus.TURN_CW ){
						control.setDriveFor(0, 0, Math.toRadians(-90), 0, Math.toRadians(-90), navigation.getPose()); // -90deg @ -30deg/s
						control.setCtrlMode(ControlMode.SETPOSE);
					}
					
					//State transition check
					currentStatus = CurrentStatus.TURN_CW;
				    lastStatus = currentStatus;
				    if (control.getCtrlMode() == ControlMode.INACTIVE) {
				    	currentStatus = CurrentStatus.INACTIVE;
				    }
						
					break;
					
				case INACTIVE:
					
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
						LCD.drawString("Pause!", 0, 0);
					}
					
					//While action
					{
						//nothing to do here
					}
					
					
					//State transition check
					lastStatus = currentStatus;
							
					if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.DRIVING0;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.INACTIVE ){
						//nothing to do here
					}
									
					break;
				case EXIT:
				
					/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
					// monitor.sendOfflineLog();
					*/
					monitor.stopLogging();
					System.exit(0);
					break;
			default:
				break;
        	}
        		
        	Thread.sleep(100);        	
		}
	}
	
	if (presentationMode == 2) {
		
	}
	
	}
	
	
	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return CTR_D3.currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception){
		LCD.clear();	
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
		
	}
	
	protected static void showData_linesensor(IPerception perception){
		LCD.clear();	
		
		LCD.drawString("left Sensor: " + perception.getLeftLineSensorValueRaw(), 0, 0);
		LCD.drawString("right Sensor: " + perception.getRightLineSensorValueRaw(), 0, 1);
		LCD.drawString("s front: " + perception.getFrontSensorDistance(), 0, 2);
		//LCD.drawString("s side: " + perception.getFrontSideSensorDistance(), 0, 3);
		
	}
}