package parkingRobot.hsamr1;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.hsamr1.Guidance_VS1.CurrentStatus;
import parkingRobot.hsamr1.ControlRST_Ver12;
import parkingRobot.hsamr1.HmiPLT_Ver12;
import parkingRobot.hsamr1.NavigationAT_Ver12;
import parkingRobot.hsamr1.PerceptionPMP_Ver12;
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
public class Guidance_VS1 {
	
	static double distanceU;
	static double distanceV;
	static double i;
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		DRIVING,
		/**
		* fast travel on long straight  
		**/
		LINE_FOLLOW_FAST,
		/**
		* slow travel in corner areas
		**/
		LINE_FOLLOW_SLOW,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
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
		
		IMonitor monitor = new Monitor_Ver12();
		
		IPerception perception = new PerceptionPMP_Ver12(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT_Ver12(perception, monitor);
		IControl    control    = new ControlRST_Ver12(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi        = new HmiPLT_Ver12(perception, navigation, control, monitor);
		
		monitor.startLogging();
				
		while(true) {
			//showData(navigation, perception);
			LCD.clear();
			
        	switch ( currentStatus )
        	{
				case LINE_FOLLOW_FAST:
					
					
					//Into action
					if ( lastStatus != CurrentStatus.LINE_FOLLOW_FAST ){
						control.setCtrlMode(ControlMode.FAST);
					}
					
					
					//While action				
					show_navigation_Data(navigation, perception);	
					
					//State transition check
					currentStatus = CurrentStatus.DRIVING;
				    lastStatus = currentStatus;
					
				    if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
						currentStatus = CurrentStatus.INACTIVE;
				    }
				   
					if(navigation.getCornerArea()==false) {
							currentStatus = CurrentStatus.LINE_FOLLOW_FAST;
						}
					if(navigation.getCornerArea()==true) {
						currentStatus=CurrentStatus.LINE_FOLLOW_SLOW;
					}
					if ( Button.ENTER.isDown() ){
	  	        		currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					    }
				    
					//Leave action
					if ( currentStatus != CurrentStatus.LINE_FOLLOW_FAST){
						//nothing to do here
					}
					
					break;
				
				case LINE_FOLLOW_SLOW:
					
					//Into action
					if ( lastStatus != CurrentStatus.LINE_FOLLOW_SLOW){
						control.setCtrlMode(ControlMode.SLOW);
					}
					

					//While action				
					show_navigation_Data(navigation, perception);	
					
					//State transition check
					currentStatus = CurrentStatus.LINE_FOLLOW_SLOW;
				    lastStatus = currentStatus;
				    
				    if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
						currentStatus = CurrentStatus.INACTIVE;
				    }
				    
				    if(navigation.getCornerArea()==false) {
						currentStatus = CurrentStatus.LINE_FOLLOW_FAST;
					}
				    if(navigation.getCornerArea()==true) {
					currentStatus=CurrentStatus.LINE_FOLLOW_SLOW;
				    }
				    if ( Button.ENTER.isDown() ){
	  	        		currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					    }
				    
					break;
				case INACTIVE:
					
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
						LCD.drawString("Pause!", 0, 0);
						//Thread.sleep(1000);
					}
					
					//While action
					show_navigation_Data(navigation, perception);
					LCD.drawString("Pause!", 0, 0);
					
					//State transition check
					lastStatus = currentStatus;
							
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
						currentStatus = CurrentStatus.LINE_FOLLOW_SLOW;		
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.LINE_FOLLOW_SLOW;
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
	
	
	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return Guidance_VS1.currentStatus;
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
	
	protected static void show_navigation_Data(INavigation navigation, IPerception perception) {
		LCD.clear();
		LCD.drawString("X   = "+ navigation.getPose().getX(), 0, 1);
		LCD.drawString("Y   = "+ navigation.getPose().getY(), 0, 2);
		LCD.drawString("phi = "+  navigation.getPose().getHeading()/Math.PI*180, 0, 3);
		LCD.drawString("s1  = "+ perception.getFrontSensorDistance(), 0, 4);
		LCD.drawString("c  = "+ navigation.getCornerDetected(), 0, 5);
		
		if(i == 0) {
			distanceU += perception.getNavigationOdo().getOdoMeasurement().getUSum();
		}
		
		LCD.drawString("U  = "+ perception.getNavigationOdo().getOdoMeasurement().getUSum(), 0, 6);
		
		if(i == 0) {
		distanceV += perception.getNavigationOdo().getOdoMeasurement().getUSum();
		}
		LCD.drawString("V   = "+perception.getNavigationOdo().getOdoMeasurement().getVSum() , 0, 7);
		i++;
		i = i%4;
	}
	
}