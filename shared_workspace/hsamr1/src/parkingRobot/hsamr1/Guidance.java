package parkingRobot.hsamr1;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;

import parkingRobot.hsamr1.ControlRST;
import parkingRobot.hsamr1.HmiPLT;
import parkingRobot.hsamr1.NavigationAT;
import parkingRobot.hsamr1.PerceptionPMP;

/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students
 * of electrical engineering with specialization 'automation, measurement and
 * control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego
 * NXT system witch can perform parking maneuvers on an predefined course. To
 * fulfill the interdisciplinary aspect of this project the software structure
 * is divided in 5 parts: human machine interface, guidance, control, perception
 * and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be
 * controlled by one or more finite state machines (FSM). It may be advantageous
 * to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be
 * realized in one main module class. Every class (except guidance) has
 * additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module
 * class thread is only handled in a synchronized context to avoid inconsistent
 * or corrupt data!
 */
public class Guidance {

	/**
	 * states for the main finite state machine. This main states are requirements
	 * because they invoke different display modes in the human machine interface.
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
		EXIT,
		/**
		 * indicates that he has get a parking slot as target
		 */
		PARK_THIS,
		/**
		 * indicates that the robot pulls out of a parking space
		 */
		PARK_OUT,
		/**
		 * indicates that the robot is parking
		 */
		PARK,
	}

	/**
	 * states for the sub finite state machine DRIVE
	 */
	public enum CurrentStatusDrive {
		/**
		 * indicates that robot is following the line fast and maybe detecting parking
		 * slots
		 */
		FAST,
		/**
		 * indicates that robot is follow the line slow and maybe detecting parking
		 * slots
		 */
		SLOW,

	}

	/**
	 * 
	 * states for the sub finite state machine PARK
	 * 
	 * @author Kirchner
	 *
	 */
	public enum CurrentStatusPark {
		/**
		 * indicates, that the robot is driving to the parking slot
		 */
		TO_SLOT,
		/**
		 * indicates that the robot reached the slot and as next step he follow the path
		 * into the slot
		 */
		REACHED_SLOT,
		/**
		 * indicates that the robot is in the slot
		 */
		IN_SLOT,
		/**
		 * indicates that the robot was in INACTIVE
		 */
		ELSE;
	}

	/**
	 * 
	 * states for the sub finite state machine PARK_OUT
	 * 
	 * @author Kirchner
	 *
	 */
	public enum CurrentStatusParkOut {
		/**
		 * indicates that the robot has to drive back
		 */
		BACKWARDS,
		/**
		 * indicates that the robot follows the path back to the line
		 */
		PARKOUT,
		/**
		 * indicates that the robot was in INACTIVE
		 */
		ELSE
	}

	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus = CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the
	 * actual state
	 */
	protected static CurrentStatus lastStatus = CurrentStatus.DRIVING;

	/**
	 * state in which the sub finite state machine DRIVE is running at the moment
	 */
	protected static CurrentStatusDrive currentStatusDrive = CurrentStatusDrive.SLOW;
	/**
	 * state in which the sub finite state machine DRIVE was running before entering
	 * the actual state
	 */
	protected static CurrentStatusDrive lastStatusDrive = CurrentStatusDrive.SLOW;

	/**
	 * state in which the sub finite state machine PARK_THIS is running at the
	 * moment
	 */
	protected static CurrentStatusPark currentStatusPark = CurrentStatusPark.TO_SLOT;
	/**
	 * state in which the sub finite state machine PARK_THIS was running before
	 * entering the actual state
	 */
	protected static CurrentStatusPark lastStatusPark = CurrentStatusPark.IN_SLOT;

	/**
	 * state in which the sub finite state machine PARK_OUT is running at the moment
	 */
	protected static CurrentStatusParkOut currentStatusParkOut = CurrentStatusParkOut.BACKWARDS;
	/**
	 * state in which the sub finite state machine PARK_OUT was running before
	 * entering the actual state
	 */
	protected static CurrentStatusParkOut lastStatusParkOut = CurrentStatusParkOut.PARKOUT;

	/**
	 * one line of the map of the robot course. The course consists of a closed
	 * chain of straight lines. Thus every next line starts where the last line ends
	 * and the last line ends where the first line starts. This documentation for
	 * line0 hold for all lines.
	 */
	static Line line0 = new Line(0, 0, 180, 0);
	static Line line1 = new Line(180, 0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30, 30, 30);
	static Line line5 = new Line(30, 30, 30, 60);
	static Line line6 = new Line(30, 60, 0, 60);
	static Line line7 = new Line(0, 60, 0, 0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight
	 * lines. Thus every next line starts where the last line ends and the last line
	 * ends where the first line starts. All above defined lines are bundled in this
	 * array and to form the course map.
	 */
	static Line[] map = { line0, line1, line2, line3, line4, line5, line6, line7 };

	/**
	 * Save the variable lastStatus
	 */
	protected static CurrentStatus lastStatusSave;
	protected static CurrentStatusPark lastStatusParkThisSave;
	protected static CurrentStatusParkOut lastStatusParkOutSave;
	private static boolean outside = false;
	private static boolean anfahrt = false;
	private static boolean correct = false;
	private static boolean back = false;
	private static int parkplatz;
	static double[] parameter;
	static Point anfahrort;
	static Point p2;
	static Pose endPose;
	static double heading;

	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus = CurrentStatus.DRIVING;

		// Generate objects

		NXTMotor leftMotor = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();

		INavigation navigation = new NavigationAT(perception, monitor);
		IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi hmi = new HmiPLT(perception, navigation, control, monitor);
		monitor.addGuidanceVar("X");
		monitor.addGuidanceVar("Y");
		monitor.addGuidanceVar("W");
		monitor.startLogging();

		while (true) {
			LCD.clear();
			LCD.drawString("S " + currentStatus, 0, 3);
			showData(navigation, perception);

			// Main finite state machine
			switch (currentStatus) {
			/////////////////////////////////////////////////////////////////
			case DRIVING:
				// Into action
				if (lastStatus != currentStatus) {
					Sound.beep();
					if (navigation.getCornerArea() == true) {
						control.setCtrlMode(ControlMode.SLOW);
					} else {
						control.setCtrlMode(ControlMode.FAST);
					}
					if (anfahrt == false) {
						navigation.setDetectionState(true);
					}
				}
				// While action
				// sub finite state machine DRIVE
				switch (currentStatusDrive) {
				case SLOW:
					// LCD.drawString("SLOW",0,1);
					if (lastStatusDrive != currentStatusDrive) {
						control.setCtrlMode(ControlMode.SLOW);
					}
					break;

				case FAST:
					// Into action
					// LCD.drawString("FAST",0,1);
					if (lastStatusDrive != currentStatusDrive) {
						control.setCtrlMode(ControlMode.FAST);
					}
					// While action
					break;
				}

				// State transition check DRIVE
				lastStatusDrive = currentStatusDrive;
				if (navigation.getCornerArea() == false) {
					currentStatusDrive = CurrentStatusDrive.FAST;
				}
				if (navigation.getCornerArea() == true) {
					currentStatusDrive = CurrentStatusDrive.SLOW;
				}

				// State transition check
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
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
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS && (anfahrt == false)) { // ausgew�hlter Parkplatz
					currentStatus = CurrentStatus.PARK_THIS;
				} else if (anfahrt == true 
							&& ( (Math.abs(navigation.getPose().getX() - anfahrort.getX()) < 0.1)
								&& (Math.abs(navigation.getPose().getY() - anfahrort.getY()) < 0.1) 
								&& (Math.abs(navigation.getPose().getHeading() - heading)    < Math.toRadians(25)) ) ){
					control.setCtrlMode(ControlMode.INACTIVE);
					Sound.twoBeeps();
					currentStatus = CurrentStatus.PARK_THIS;
					currentStatusPark = CurrentStatusPark.REACHED_SLOT;
				}

				// Leave action
				if (currentStatus != CurrentStatus.DRIVING) {
					navigation.setDetectionState(false);
				}
				break;
			/////////////////////////////////////////////////////////////////////////////////
			case INACTIVE:
				// Into action
				if (lastStatus != CurrentStatus.INACTIVE) {
					control.setCtrlMode(ControlMode.INACTIVE);
					lastStatusSave = lastStatus;
					if(lastStatusSave==CurrentStatus.PARK_THIS) {
						lastStatusParkThisSave=currentStatusPark;
					}else if(lastStatusSave==CurrentStatus.PARK_OUT) {
						lastStatusParkOutSave=currentStatusParkOut;
					}
				}

				// While action

				// State transition check
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT) {
					currentStatus = lastStatusSave;
					if(lastStatusSave==CurrentStatus.PARK_THIS) {
						currentStatusPark=lastStatusParkThisSave;
						lastStatusPark=CurrentStatusPark.ELSE;
					}else if(lastStatusSave==CurrentStatus.PARK_OUT) {
						currentStatusParkOut=lastStatusParkOutSave;
						lastStatusParkOut=CurrentStatusParkOut.ELSE;
					}
				} else if (Button.ENTER.isDown()) {
					currentStatus = lastStatusSave;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}

				// Leave action

				break;
			////////////////////////////////////////////////////////////////////////////////////////
			case EXIT:
				hmi.disconnect();
				/**
				 * NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE) //
				 * monitor.sendOfflineLog();
				 */
				// monitor.stopLogging();
				System.exit(0);
				break;
			////////////////////////////////////////////////////////////////////////////////////////
			case PARK:
				//
				// Into action
				if (lastStatus != CurrentStatus.PARK) {
					control.setCtrlMode(ControlMode.INACTIVE);
					Sound.twoBeeps();
				}
				// While action
				LCD.drawString("Abstand" + perception.getFrontSensorDistance(), 0, 5);
				LCD.drawString("Abstand" + perception.getBackSensorDistance(), 0, 6);
				// State transition check
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_OUT) { // PARKOUT
					currentStatus = CurrentStatus.PARK_OUT; // PARK_OUT
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
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
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}
				// Leave action

				//
				break;
			////////////////////////////////////////////////////////////////////////////////////////
			case PARK_THIS:
				LCD.drawString("SP " + currentStatusPark, 0, 4);
				// Into action
				if (lastStatus != CurrentStatus.PARK_THIS) {
					if (anfahrt == false && (lastStatus != CurrentStatus.INACTIVE)) {
						currentStatusPark = CurrentStatusPark.TO_SLOT;
					}
					parkplatz = hmi.getSelectedParkingSlot();
					ParkingSlot[] parkingslots = navigation.getParkingSlots();
					anfahrort = parkingslots[parkplatz - 1].getFrontBoundaryPosition(); // start
					p2 = parkingslots[parkplatz - 1].getBackBoundaryPosition();// ende
					anfahrt = false;
					correct = false;
				}
				// While action
				// sub finite state machine PARK_THIS
				switch (currentStatusPark) {
				case TO_SLOT:
					anfahrt = true;
					
						//Winkelberechnung
						if(Math.abs(anfahrort.getX()-p2.getX())<0.05){ //deltaX small -> Angle 90�
							heading=Math.PI/2;
						}else if((anfahrort.getY()-p2.getY())<0.05) { //deltaY small -> Angle 180� or 0�
							if(anfahrort.getX()<p2.getX()) {//if startpose global smaller than endpose -> 0� 
								heading=0;
							}else if(anfahrort.getX()>p2.getX()) {//If startpose global greater than endpose -> 180�
								heading=Math.PI;
							}
						}
					
					
					if (Math.abs(heading) < Math.toRadians(20)) { //0�
						anfahrort.x = anfahrort.x + 0.05f;
					} else if (Math.abs(heading - Math.PI / 2) < Math.toRadians(20)) { //90�
						anfahrort.y = anfahrort.y + 0.1f;
					} else if (Math.abs(heading - Math.PI) < Math.toRadians(20)) { //180�
						anfahrort.x = anfahrort.x - 0.08f;		// geaendert auf 
					}

					currentStatus = CurrentStatus.DRIVING;
					break;
				////////////////////////////////
				case REACHED_SLOT:
					// Into-action
					if (lastStatusPark != CurrentStatusPark.REACHED_SLOT) {
						Pose startPose = navigation.getPose();
						if(lastStatus!=CurrentStatus.INACTIVE) {
						if (Math.abs(startPose.getHeading()) < Math.toRadians(20)) {
							startPose.setHeading((float) Math.toRadians(0));
							endPose = new Pose(startPose.getX() + 0.40f, startPose.getY() - 0.25f, 0);
						} else if (Math.abs(startPose.getHeading() - Math.PI / 2) < Math.toRadians(20)) {
							startPose.setHeading((float) Math.toRadians(90));
							endPose = new Pose(startPose.getX() + 0.25f, startPose.getY() + 0.35f, (float) Math.toRadians(90));
						} else if (Math.abs(startPose.getHeading() - Math.PI) < Math.toRadians(20)) {
							startPose.setHeading((float) Math.toRadians(180));
							endPose = new Pose((startPose.getX() - 0.40f), startPose.getY() + 0.20f, (float) Math.toRadians(180));
						}
						
						
						control.setVelocity(10);
						control.setParkingFor(startPose, endPose);
						}
						control.setCtrlMode(ControlMode.PARK_CTRL);
					}

					// While-action

					if (control.getCtrlMode() == ControlMode.INACTIVE) {

						currentStatusPark = CurrentStatusPark.IN_SLOT;
					}
					break;
				////////////////////////////////
				case IN_SLOT:
					correct = true;
					break;
				////////////////////////////////

				}

				// State transition check
				lastStatus = currentStatus;
				lastStatusPark = currentStatusPark;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
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
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (control.getCtrlMode() == ControlMode.INACTIVE && correct) {
					currentStatus = CurrentStatus.PARK;
					Thread.sleep(500);
				}
				// Leave action
				break;
			////////////////////////////////////////////////////////////////////////////////////////
			case PARK_OUT:
				// Into action

				if (lastStatus != CurrentStatus.PARK_OUT) {
					if(lastStatus!=CurrentStatus.INACTIVE) {
					currentStatusParkOut = CurrentStatusParkOut.BACKWARDS;
					}
					back = false;
				}

				// While action
				// sub finite state machine PARK_OUT
				switch (currentStatusParkOut) {

				case BACKWARDS:
					
					if (lastStatusParkOut != currentStatusParkOut) {
						if(lastStatus!=CurrentStatus.INACTIVE) {
							if (perception.getFrontSensorDistance() < 30) {
								double distance = perception.getBackSensorDistance();
								distance = distance - 1;

								if ((Math.abs(Math.toRadians(90) - navigation.getPose().getHeading()) < Math
										.toRadians(20))) {// wenn Winkel 90�
									control.setDriveFor(0, distance * (-0.01), 0, -10, 0, navigation.getPose());
								} else if ((Math.abs(navigation.getPose().getHeading()) < Math.toRadians(20))) { // wenn Winkel 0�
									control.setDriveFor(distance * (-0.01), 0, 0, -10, 0, navigation.getPose());
								} else {// Winkel 180�
									control.setDriveFor((distance * 0.01), 0, 0, -10, 0, navigation.getPose());
								}
							} else {
								if ((Math.abs(Math.toRadians(90) - navigation.getPose().getHeading()) < Math
										.toRadians(20))) {// wenn Winkel 90�
									control.setDriveFor(0, -0.2, 0, -10, 0, navigation.getPose());
								} else if ((Math.abs(navigation.getPose().getHeading()) < Math.toRadians(20))) { // wenn
																												// Winkel
																												// 0�
									control.setDriveFor(-0.2, 0, 0, -10, 0, navigation.getPose());
								} else {// Winkel 180�
									control.setDriveFor(0.2, 0, 0, -10, 0, navigation.getPose());
								}
							}
						}else if(lastStatus==CurrentStatus.INACTIVE) {
							Sound.twoBeeps();
							if ((Math.abs(Math.toRadians(90) - navigation.getPose().getHeading()) < Math.toRadians(20))) {// wenn Winkel 90�
								control.setDriveFor(0, -0.2, 0, -10, 0, navigation.getPose());
							} else if ((Math.abs(navigation.getPose().getHeading()) < Math.toRadians(20))) { // wenn Winkel 0�
								control.setDriveFor(-0.2, 0, 0, -10, 0, navigation.getPose());
							} else {// Winkel 180�
								control.setDriveFor(0.2, 0, 0, -10, 0, navigation.getPose());
							}
						}
						control.setCtrlMode(ControlMode.SETPOSE);
						}

						Thread.sleep(50);
					
				
					if (control.getCtrlMode() == ControlMode.INACTIVE) {
						if(perception.getBackSensorDistance()==9) {
							back = true;
						}	
					}
					break;
				/////////////////////////////////////////////////////////////////////////////////////////////////////
				case PARKOUT:
					if (lastStatusParkOut != currentStatusParkOut) {
						if(lastStatusParkOut!=CurrentStatusParkOut.ELSE) {
							Pose startPose = navigation.getPose();
							if (Math.abs(startPose.getHeading()) < Math.toRadians(20)) {
								startPose.setHeading(0);
								endPose = new Pose(startPose.getX() + 0.40f, startPose.getY() + 0.25f, 0);
							} else if (Math.abs(startPose.getHeading() - Math.PI / 2) < Math.toRadians(20)) {
								startPose.setHeading((float) Math.toRadians(90));
								endPose = new Pose(startPose.getX() - 0.25f, startPose.getY() + .35f, (float) Math.toRadians(90));
							} else if (Math.abs(startPose.getHeading() - Math.PI) < Math.toRadians(20)) {
								startPose.setHeading((float) Math.toRadians(180));
								endPose = new Pose(startPose.getX() - 0.40f, startPose.getY() - 0.225f, (float) Math.toRadians(180));
							}
							control.setVelocity(10);
							control.setParkingFor(startPose, endPose);
						}
						control.setCtrlMode(ControlMode.PARK_CTRL);
					}
					if (control.getCtrlMode() == ControlMode.INACTIVE) {
						outside = true;
					}
					break;
				}
				// State transition check 1
				lastStatusParkOut = currentStatusParkOut;
				if (back == true) {
					currentStatusParkOut = CurrentStatusParkOut.PARKOUT;
					back = false;
				}

				// State transition check 2
				lastStatus = currentStatus;

				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) { 
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
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (control.getCtrlMode() == ControlMode.INACTIVE && outside) {
					currentStatus = CurrentStatus.DRIVING;
					outside = false;
				}

				// Leave action

				//
				break;
			}
			Thread.sleep(100);
		}
	}

	/**
	 * returns the actual state of the main finite state machine as defined by the
	 * requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus() {
		return Guidance.currentStatus;
	}
	
	/**
	 * return true if "Anfahrt" true
	 * @return anfahrt
	 */
	public static boolean getAnfahrt() {
		return Guidance.anfahrt;
	}

	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose
	 *                   information
	 * @param perception reference to the perception class for sensor information                  
	 */
	protected static void showData(INavigation navigation, IPerception perception) {

		LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);

	}

}