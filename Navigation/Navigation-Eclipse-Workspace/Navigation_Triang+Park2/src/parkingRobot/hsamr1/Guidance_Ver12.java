package parkingRobot.hsamr1;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.hsamr1.ControlRST_Ver12;
import parkingRobot.hsamr1.HmiPLT_Ver12;
import parkingRobot.hsamr1.NavigationAT_Ver12;
import parkingRobot.hsamr1.PerceptionPMP_Ver12;
import parkingRobot.hsamr1.Guidance_Ver12.CurrentStatus;
import parkingRobot.hsamr1.Guidance_Ver12.CurrentStatusDrive;


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
public class Guidance_Ver12 {
	
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
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}
	/**
	 * states for the sub finite state machine.
	 */
		
public enum CurrentStatusDrive {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		FAST,
		/**
		 * indicates that robot is performing an parking
		 */
		SLOW,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		TURN
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
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatusDrive currentStatusDrive 	= CurrentStatusDrive.SLOW;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatusDrive lastStatusDrive		= CurrentStatusDrive.SLOW;
	
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
	
	
	private static boolean turning=false;
	
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
			LCD.clear();
			show_navigation_Data(navigation, perception);
        	switch ( currentStatus )
        	{
				case DRIVING:
					// MONITOR (example)
//					monitor.writeGuidanceComment("Guidance_Driving");
						
					//LCD.drawString("DRIVING",0,0);
					//Into action
					if(lastStatus!=currentStatus) {
						control.setCtrlMode(ControlMode.SLOW);
						navigation.setDetectionState(true);
					}
					//While action	
					switch(currentStatusDrive)
					{
					case SLOW:
						//LCD.drawString("SLOW",0,1);
							if(lastStatusDrive!=currentStatusDrive) {
								control.setCtrlMode(ControlMode.SLOW);
							}
							break;
							
					case FAST:
							//Into action
						//LCD.drawString("FAST",0,1);
							if(lastStatusDrive!=currentStatusDrive) {
								control.setCtrlMode(ControlMode.FAST);		
							}
							//While action														
							break;					
			}
					
					//State transition check DRIVE
					lastStatusDrive = currentStatusDrive;
					if(navigation.getCornerArea()==false || turning) {
						currentStatusDrive=CurrentStatusDrive.FAST;						
						turning=false;
					}
					
					if(navigation.getCornerArea()==true && (currentStatusDrive!=CurrentStatusDrive.TURN)) {
						currentStatusDrive=CurrentStatusDrive.SLOW;
					}
					/*
					if(navigation.getCornerArea()==true && navigation.getCorner()==true) {
						currentStatusDrive=CurrentStatusDrive.TURN;
					}
					*/	

					//State transition check
					lastStatus = currentStatus;
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
						currentStatus = CurrentStatus.INACTIVE;
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING ){
						navigation.setDetectionState(false);
					}
					break;				
				case INACTIVE:
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					
					//While action
					{
						//nothing to do here
					}
					
					//State transition check
					lastStatus = currentStatus;
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
						currentStatus = CurrentStatus.DRIVING;						
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.DRIVING;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.INACTIVE ){
						//nothing to do here
					}					
					break;
				case EXIT:
					//hmi.disconnect();
					/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
					// monitor.sendOfflineLog();
					*/
					//monitor.stopLogging();
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
		return Guidance_Ver12.currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception){	
		
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		//LCD.drawString("Slots:): " + (navigation.getParkingslotsAmount()), 0, 2);
		
		
//		perception.showSensorData();
		
//    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
//			LCD.drawString("HMI Mode SCOUT", 0, 3);
//		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
//			LCD.drawString("HMI Mode PAUSE", 0, 3);
//		}else{
//			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
//		}
		
	}
	protected static void show_navigation_Data(INavigation navigation, IPerception perception) {
		LCD.clear();
		LCD.drawString("X   = "+ navigation.getPose().getX(), 0, 1);
		LCD.drawString("Y   = "+ navigation.getPose().getY(), 0, 2);
		LCD.drawString("s1  = "+ perception.getFrontSideSensorDistance(), 0, 3);
		LCD.drawString("Slots  = "+ navigation.getNumberOfSlots(), 0, 4);
		//LCD.drawString("phi = "+  navigation.getPose().getHeading()/Math.PI*180, 0, 4);
		//LCD.drawString("lcn  = "+ navigation.getLastCornerNumber(), 0, 5);
		//LCD.drawString("ncn  = "+ navigation.getNextCornerNumber(), 0, 6);
		//LCD.drawString("ci   = "+ navigation.getCornerIndex(), 0, 7);
		
		
	}
	
		
}