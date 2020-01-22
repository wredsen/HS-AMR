package parkingRobot.hsamr1;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation3;
import parkingRobot.INavigation3.ParkingSlot;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;
import parkingRobot.INavigation2;
import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;

import parkingRobot.hsamr1.ControlRST;
import parkingRobot.hsamr1.HmiPLT;
import parkingRobot.hsamr1.NavigationAT3;
import parkingRobot.hsamr1.PerceptionPMP;
import parkingRobot.hsamr1.GuiDemo.CurrentStatus;
import parkingRobot.hsamr1.GuiDemo.CurrentStatusDrive;





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
public class GuiDemo {
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		EINPARK,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		AUSPARK,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EINPARK_2,
		/**
		 * indicates that he has get a parking slot as target
		 * <br>
		 * Zeigt an, dass der Roboter ein Parkplatz als Ziel besitzt.
		 */
		AUSPARK_2,
		/**
		 * Zeigt an, dass der Roboter ausparkt.
		 */
		LINE_FOLLOW,
		/**
		 * Zeigt an, dass der Roboter in der Parklücke parkt.
		 */
		RUECKWAERTS,
		
		LINE_FOLLOW_2,
		
		ELSE,
		
		INACTIVE,
		
		EXIT,
		
		DRIVING_1,
		DRIVING_2,
		DREHUNG_1,
		DREHUNG_2,
		DREHUNG_3,
		PARK,
	}
	/**
	 * states for the sub finite state machine DRIVE
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
		
	}

	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.EINPARK;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.ELSE;

	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatusDrive currentStatusDrive 	= CurrentStatusDrive.FAST;
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
	protected static CurrentStatus a;
	private static boolean outside=false;
	private static boolean anfahrt=false;
	private static boolean correct=false;
	private static boolean back=false;
	private static int parkplatz;
	static double[] parameter;
	static Point anfahrort;  
	static Point p2;
	private static float differenz=0;
	static Pose endPose;
	private static double heading=0;
	static private int demomode=0;
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {		
        currentStatus = CurrentStatus.EINPARK;
        lastStatus    = CurrentStatus.ELSE;
		
		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);
		
		IMonitor monitor = new Monitor();
		
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();
		
		INavigation3 navigation3 = new NavigationAT3(perception, monitor);
		INavigation2 navigation2 = new NavigationAT2(perception, monitor);
		IControl    control3    = new ControlRST(perception, navigation3, leftMotor, rightMotor, monitor);
		IControl	control2 	= new ControlRST(perception, navigation2, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi3        = new HmiPLT(perception, navigation3, control3, monitor);
		INxtHmi  	hmi2        = new HmiPLT(perception, navigation2, control2, monitor);
		
		monitor.addGuidanceVar("X");
		monitor.addGuidanceVar("Y");
		monitor.addGuidanceVar("W");
		monitor.startLogging();
		
		demomode=setDemoMode();
		
		
				
		while(demomode==3) {
			LCD.clear();
			LCD.drawString("S"+currentStatus, 0, 5);	
        	switch ( currentStatus )
        	{
        	/////////////////////////////////////////////////////////////////
			case EINPARK:
				//erste Parklucke
				if(currentStatus!=lastStatus) {
					Pose startPose = navigation3.getPose();
					Pose endPose = new Pose(0.60f, -0.25f, 0f);
					control3.setDriveFor(0, 0, 0, 10, 0, navigation3.getPose());
					control3.setParkingFor(startPose,endPose);
					control3.setCtrlMode(ControlMode.PARK_CTRL);
					lastStatus=currentStatus;
					Thread.sleep(50);						
				}
				if(control3.getCtrlMode()==ControlMode.INACTIVE) {
					currentStatus=CurrentStatus.AUSPARK;
					Thread.sleep(500);
				}
				break;
			/////////////////////////////////////////////////////////////////
			case AUSPARK:
				if(currentStatus!=lastStatus) {
					Pose startPose = navigation3.getPose();
					Pose endPose = new Pose(navigation3.getPose().getX()+0.60f,navigation3.getPose().getY()+0.25f, 0f);
					control3.setDriveFor(0, 0, 0, 10, 0, navigation3.getPose());
					control3.setParkingFor(startPose,endPose);
					control3.setCtrlMode(ControlMode.PARK_CTRL);
					lastStatus=currentStatus;
											
				}
				if(control3.getCtrlMode()==ControlMode.INACTIVE) {
					anfahrt=true;
					anfahrort = new Point(1.80f, 0.1f);
					heading = Math.PI/2;
					currentStatus=CurrentStatus.LINE_FOLLOW;
					Thread.sleep(500);
				}
				break;
			/////////////////////////////////////////////////////////////////
			case LINE_FOLLOW:
				// Into action
				if (lastStatus != currentStatus) {
					Sound.beep();
					if (navigation3.getCornerArea() == true) {
						control3.setCtrlMode(ControlMode.SLOW);
					} else {
						control3.setCtrlMode(ControlMode.FAST);
					}
					navigation3.setDetectionState(true);
				}
				// While action
				// sub finite state machine DRIVE
				switch (currentStatusDrive) {
				case SLOW:
					// LCD.drawString("SLOW",0,1);
					if (lastStatusDrive != currentStatusDrive) {
						control3.setCtrlMode(ControlMode.SLOW);
					}
					break;

				case FAST:
					// Into action
					// LCD.drawString("FAST",0,1);
					if (lastStatusDrive != currentStatusDrive) {
						control3.setCtrlMode(ControlMode.FAST);
					}
					// While action
					break;
				}

				// State transition check DRIVE
				lastStatusDrive = currentStatusDrive;
				if (navigation3.getCornerArea() == false) {
					currentStatusDrive = CurrentStatusDrive.FAST;
				}
				if (navigation3.getCornerArea() == true) {
					currentStatusDrive = CurrentStatusDrive.SLOW;
				}

				// State transition check
				lastStatus = currentStatus;
				if (hmi3.getMode() == parkingRobot.INxtHmi.Mode.PAUSE && (anfahrt != true)) {
					currentStatus = CurrentStatus.INACTIVE;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.INACTIVE;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				//} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					//	currentStatus = CurrentStatus.EXIT;
				} else if (anfahrt == true 
						&& ( (Math.abs(navigation3.getPose().getX() - anfahrort.getX()) < 0.05)
							&& (Math.abs(navigation3.getPose().getY() - anfahrort.getY()) < 0.05) 
							&& (Math.abs(navigation3.getPose().getHeading() - heading)    < Math.toRadians(15)) ) ){
					Sound.twoBeeps();
					control3.setCtrlMode(ControlMode.INACTIVE);
					currentStatus = CurrentStatus.EINPARK_2;
					Thread.sleep(400);
				}

				// Leave action
				if (currentStatus != CurrentStatus.LINE_FOLLOW) {
					navigation3.setDetectionState(false);
				}
				break;
			/////////////////////////////////////////////////////////////////
			case EINPARK_2:
				//180°-  max. °/s math. pos
				if(currentStatus!=lastStatus) {
					Pose startPose = navigation3.getPose();
					startPose.setHeading((float) Math.toRadians(90));
					Pose endPose = new Pose(navigation3.getPose().getX()+0.25f,navigation3.getPose().getY()+0.30f, (float) Math.PI/2);
					control3.setDriveFor(0, 0, 0, 10, 0, navigation3.getPose());
					control3.setParkingFor(startPose,endPose);
					control3.setCtrlMode(ControlMode.PARK_CTRL);
					lastStatus=currentStatus;
					Thread.sleep(50);						
				}
				if(control3.getCtrlMode()==ControlMode.INACTIVE) {
					currentStatus=CurrentStatus.RUECKWAERTS;
					Thread.sleep(500);
					Sound.beep();
				}
				break;
			//////////////////////////////////////////////////////////////////
			case RUECKWAERTS:
				if(currentStatus!=lastStatus) {
					lastStatus=currentStatus;
					double distance = perception.getBackSensorDistance();
					distance = distance - 3;
					control3.setDriveFor(0,-0.01*distance,0,-10, 0, navigation3.getPose());
					control3.setCtrlMode(ControlMode.SETPOSE);
				}
				if(control3.getCtrlMode()==ControlMode.INACTIVE) {
					currentStatus=CurrentStatus.AUSPARK_2;
					Thread.sleep(500);
					Sound.beep();
				}
				break;
			//////////////////////////////////////////////////////////////////////
			case AUSPARK_2:
				if(currentStatus!=lastStatus) {
					Pose startPose = navigation3.getPose();
					startPose.setHeading((float) Math.toRadians(90));
					Pose endPose = new Pose(navigation3.getPose().getX()-0.25f, navigation3.getPose().getY()+0.30f, (float)Math.PI/2);
					control3.setDriveFor(0, 0, 0, 10, 0, navigation3.getPose());
					control3.setParkingFor(startPose,endPose);
					control3.setCtrlMode(ControlMode.PARK_CTRL);
					lastStatus=currentStatus;
											
				}
				if(control3.getCtrlMode()==ControlMode.INACTIVE) {
					currentStatus=CurrentStatus.LINE_FOLLOW;
					outside=true;
					Sound.twoBeeps();
					Thread.sleep(500);		
				}
				break;
				/////////////////////////////////////////////////////////////////////////////////	
			case INACTIVE:
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
							control3.setCtrlMode(ControlMode.INACTIVE);
							a = lastStatus;
					}

					//While action


					//State transition check
					lastStatus = currentStatus;
					if ( hmi3.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
						currentStatus = a;						
					}else if ( Button.ENTER.isDown() ){
						currentStatus = a;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi3.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;
					}

					//Leave action

					break;
			////////////////////////////////////////////////////////////////////////////////////////
			case EXIT:
				hmi3.disconnect();
				System.exit(0);
				break;
        	}		
        	Thread.sleep(100);
		}
		
		
		while (demomode==2) {
			LCD.clear();
			//showData(navigation2, perception);

			switch (currentStatus) {
			/////////////////////////////////////////////////////////////////
			case DRIVING_1:
				// 120 - 10 cm/s
				if (currentStatus != lastStatus) {
					control2.setDriveFor(1.20, 0, 0, 10, 0, navigation2.getPose());
					control2.setCtrlMode(ControlMode.SETPOSE);
					lastStatus = CurrentStatus.DRIVING_1;
					Thread.sleep(50);
				}
				if (control2.getCtrlMode() == ControlMode.INACTIVE) {
					currentStatus = CurrentStatus.DREHUNG_1;
					Thread.sleep(500);
					Sound.beep();
				}
				break;
			/////////////////////////////////////////////////////////////////
			case DREHUNG_1:
				// 90°- 15 °/s math. pos
				if (currentStatus != lastStatus) {
					control2.setDriveFor(0, 0, Math.toRadians(90), 0, Math.toRadians(60), navigation2.getPose());
					control2.setCtrlMode(ControlMode.SETPOSE);
					lastStatus = CurrentStatus.DREHUNG_1;

				}
				if (control2.getCtrlMode() == ControlMode.INACTIVE) {
					currentStatus = CurrentStatus.DRIVING_2;
					Thread.sleep(500);
					Sound.beep();
				}
				break;
			/////////////////////////////////////////////////////////////////
			case DRIVING_2:
				// 30 cm - 5 cm/s
				if (currentStatus != lastStatus) {
					control2.setDriveFor(0, 0.27, 0, 10, 0, navigation2.getPose());
					Thread.sleep(50);
					control2.setCtrlMode(ControlMode.SETPOSE);
					lastStatus = currentStatus;
				}
				if (control2.getCtrlMode() == ControlMode.INACTIVE) {
					currentStatus = CurrentStatus.DREHUNG_2;
					Thread.sleep(500);
					Sound.beep();
				}
				break;
			/////////////////////////////////////////////////////////////////
			case DREHUNG_2:
				// 90°- 15 °/s math. pos
				if (currentStatus != lastStatus) {
					control2.setDriveFor(0, 0, Math.toRadians(-90), 0, Math.toRadians(-60), navigation2.getPose());
					control2.setCtrlMode(ControlMode.SETPOSE);
					lastStatus = CurrentStatus.DREHUNG_2;
					Thread.sleep(50);
				}
				if (control2.getCtrlMode() == ControlMode.INACTIVE) {
					currentStatus = CurrentStatus.LINE_FOLLOW;
					
					Thread.sleep(500);
					anfahrt=true;
					anfahrort= new Point(0.07f,0f);
					heading = Math.PI;
					Sound.twoBeeps();
				}
				break;
			/////////////////////////////////////////////////////////////////
			case LINE_FOLLOW:
				// Into action
				if (lastStatus != currentStatus) {
					//Sound.beep();
					if (navigation2.getCornerArea() == true) {
						control2.setCtrlMode(ControlMode.SLOW);
					} else {
						control2.setCtrlMode(ControlMode.FAST);
					}
					navigation2.setDetectionState(true);
				}
				// While action
				// sub finite state machine DRIVE
				switch (currentStatusDrive) {
				case SLOW:
					// LCD.drawString("SLOW",0,1);
					if (lastStatusDrive != currentStatusDrive) {
						control2.setCtrlMode(ControlMode.SLOW);
					}
					break;

				case FAST:
					// Into action
					// LCD.drawString("FAST",0,1);
					if (lastStatusDrive != currentStatusDrive) {
						control2.setCtrlMode(ControlMode.FAST);
					}
					// While action
					break;
				}

				// State transition check DRIVE
				lastStatusDrive = currentStatusDrive;
				if (navigation2.getCornerArea() == false) {
					currentStatusDrive = CurrentStatusDrive.FAST;
				}
				if (navigation2.getCornerArea() == true) {
					currentStatusDrive = CurrentStatusDrive.SLOW;
				}

				// State transition check
				lastStatus = currentStatus;
				if (hmi2.getMode() == parkingRobot.INxtHmi.Mode.PAUSE && (anfahrt != true)) {
					currentStatus = CurrentStatus.INACTIVE;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.INACTIVE;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				//} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					//	currentStatus = CurrentStatus.EXIT;
				} else if (anfahrt == true 
							&& ( (Math.abs(navigation2.getPose().getX() - anfahrort.getX()) < 0.05)
								&& (Math.abs(navigation2.getPose().getY() - anfahrort.getY()) < 0.05) 
								&& (Math.abs(-navigation2.getPose().getHeading() - heading)    < Math.toRadians(15)) ) ) {
					control2.setCtrlMode(ControlMode.INACTIVE);
					currentStatus = CurrentStatus.DREHUNG_3;
					Sound.twoBeeps();
					Thread.sleep(400);
				}

				// Leave action
				if (currentStatus != CurrentStatus.LINE_FOLLOW) {
					navigation2.setDetectionState(false);
				}
				break;
			/////////////////////////////////////////////////////////////////
			case DREHUNG_3:
				// 180°- max. °/s math. pos
				if (currentStatus != lastStatus) {
					control2.setDriveFor(0, 0, Math.toRadians(180), 0, Math.toRadians(120), navigation2.getPose());
					control2.setCtrlMode(ControlMode.SETPOSE);
					lastStatus = CurrentStatus.DREHUNG_3;
					Thread.sleep(50);
				}
				if (control2.getCtrlMode() == ControlMode.INACTIVE) {
					currentStatus = CurrentStatus.PARK;
					Thread.sleep(500);
				}
				break;
			//////////////////////////////////////////////////////////////////
			case PARK:
				if (currentStatus != lastStatus) {
					endPose = new Pose(navigation2.getPose().getX() + 0.60f, navigation2.getPose().getY() - .26f, 0);
					control2.setDriveFor(0, 0, 0, 10, 0, navigation2.getPose());
					control2.setParkingData(navigation2.getPose(), endPose);
					control2.setCtrlMode(ControlMode.PARK_CTRL);
					lastStatus=currentStatus;
				}
				if (control2.getCtrlMode() == ControlMode.INACTIVE) {
					currentStatus = CurrentStatus.INACTIVE;
					Thread.sleep(500);
				}
				break;
			//////////////////////////////////////////////////////////////////////
			case INACTIVE:
				//Into action
				if ( lastStatus != CurrentStatus.INACTIVE ){
						control2.setCtrlMode(ControlMode.INACTIVE);
						a = lastStatus;
				}

				//While action


				//State transition check
				lastStatus = currentStatus;
				if ( hmi2.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
					currentStatus = a;						
				}else if ( Button.ENTER.isDown() ){
					currentStatus = a;
					while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
				}else if ( Button.ESCAPE.isDown() ){
					currentStatus = CurrentStatus.EXIT;
					while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
				}else if (hmi2.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
					currentStatus = CurrentStatus.EXIT;
				}

				//Leave action

				break;
		////////////////////////////////////////////////////////////////////////////////////////
		case EXIT:
			hmi2.disconnect();
			System.exit(0);
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
		return GuiDemo.currentStatus;
	}
	
	/**
	 * return true if "Anfahrt" true
	 * @return anfahrt
	 */
	public static boolean getAnfahrt() {
		return GuiDemo.anfahrt;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation3 navigation, IPerception perception){	
		
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
		
	}

	protected static int setDemoMode() {
		int a = 0;
		LCD.clear();
		LCD.drawString("Auswahl", 0, 0);
		LCD.drawString("Demoprogramme", 0, 1);
		LCD.drawString("Demo2 Links", 0, 2);
		LCD.drawString("Demo3 Rechts.", 0, 3);
		while(true) {
			if(Button.LEFT.isDown()) {
				return 2;
				
			}else if(Button.RIGHT.isDown()) {
				return 3;
			}
		}
	}
}