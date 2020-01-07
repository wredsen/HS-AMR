package parkingRobot.hsamr1;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.hsamr1.NavigationThread;
import parkingRobot.hsamr1.Guidance;
import parkingRobot.hsamr1.Guidance.CurrentStatus;
import parkingRobot.IMonitor;

import java.util.*;

/**
 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation{
	
	INavigation.ParkingSlot[] Slots=null;  
	int mapModus = 1; 	// 1 for real, big map and 2 for small test-map
	
	/////////////////////////////////////////////////////////// Variablen fuer Parkluecken
	boolean foundBackBoundary = false;
	boolean verticalSlot = false;
	
	Point newBackBoundaryPosition=null;
	Point newFrontBoundaryPosition=null;
	
	int parkingSlotAreaNumber = 0;
	int numberOfSlots = 0;
	int ID = 1;
	int slotListIndex = 0;
	int slotMeasurementQuality = 1;
	ArrayList<ParkingSlot>slotList=new ArrayList<ParkingSlot>();
	
	////////////////////////////////////////////////////////////	Variabeln fuer Lokalisierungsauswertung
	boolean cornerDetect;
	double angleDiff = 0;													
	double yDiff = 0;
	double xDiff = 0;
	//////////////////////////////////////////////////////////
	double testvariable1 = 0;
	double testvariable2 = 0;
	double testvariable3 = 0;
	
	/**
	 * holds the value for the average difference of the sensor that DOES NOT go over the black line in a Corner
	 */
	double lowLimit = 15;
	
	/**
	 * holds the value for the average difference of the sensor that DOES go over the black line in a Corner
	 */
	
	double highLimit = -60;
	
	
	/**
	 * Linked List for right light sensor values
	 */
	LinkedList<Float> angleList = new LinkedList<Float>();	
	
	/**
	 * color information measured by right light sensor: 0% --> black , 100% --> white
	 */
	int rightLightSensorValue = 0;
	
	/**
	 * color information measured by left light sensor: 0% --> black , 100% --> white
	 */
	int leftLightSensorValue = 0;
	
	/**
	 * holds the index number of the last corner
	 */
	private int lastCornerNumber = 0;
	
	/**
	 * holds the index number of the next corner
	 */
	private int nextCornerNumber = 1;
	
	/**
	 * holds the index number of the current corner area
	 */
	private int cornerIndexNumber = 0;
	
	/**
	 * holds the index number of the current corner area
	 */
	private int fastAreaIndex = 0;
	
	/**
	* save the last angle check
	*/
	private int lastcheck = 0;
	
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement
	 * between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between
	 * actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensor pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;


	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.15; // only rough guess, to be measured exactly and maybe refined by experiments

	
	/**
	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	Line[] map 								= null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();

	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();		
		
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
		
		monitor.addNavigationVar("diff");
		monitor.addNavigationVar("nextCornerNumber");
		monitor.addNavigationVar("lastCornerNumber");
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = map;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		this.updateSensors();
		

		if (this.parkingSlotDetectionIsOn) {
				this.detectParkingSlot();		
		}
		
		this.calculateLocation();
		this.getCorner();
		
		//calculate index 
		cornerIndexNumber = cornerIndexNumber % 8;
		nextCornerNumber = nextCornerNumber % 8;
		lastCornerNumber = lastCornerNumber % 8;
		fastAreaIndex = fastAreaIndex % 4;
		
		//check methods to reset location
		checkAreaIndex();
		checkAngel();
				
		//monitor.writeNavigationVar("cornerNumber", "" + this.nextCornerNumber);	
		//monitor.writeNavigationVar("nextCornerNumber", "" + this.lastCornerNumber);	
	}
	
	
	// Outputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		return Slots;
	}
	
	public synchronized int getNumberOfSlots() {
		return numberOfSlots;
	}
	
	public synchronized boolean getCorner() {
		
		//check if in corner area and if it is a new area
		if (getCornerArea() == true && cornerIndexNumber == nextCornerNumber) {
			
			
			    //special condition for corner 4
				if(cornerIndexNumber == 4 && this.getPose().getHeading()>=2.8 && this.getPose().getHeading() <= 3.5) {
					lastCornerNumber++;
				}
			
				//normal condition with sharp sensor
				if(detectCorner() && cornerIndexNumber != 4 && cornerDetect == false) {
					lastCornerNumber++;
					cornerDetect = true;
					monitor.writeNavigationComment("Test");
				}
				
				//override pose depend on current position 
				if(lastCornerNumber == nextCornerNumber && getAngleCrit()) {
					evaluateCornerDetection();
					cornerDetect = false;
					//Sound.beep();
				
				return true;
				}
			else return false;
		}
		
		return false;

	}
	
	/**
	 * detects corner, determines type of corner and returns boolean value: corner detected --> true ; no corner detected --> false
	 */
	public boolean detectCorner() {
		
		if (perception.getFrontSensorDistance() < 19 && lastCornerNumber != 4) {
			return true;
		}
		else if(perception.getFrontSensorDistance() < 55 && nextCornerNumber == 5){
			return true;
		}
		else {
			return false;
		}
	}
	
	/**
	 * checks if the robot has finished the rotation in the corners
	 */
	public boolean getAngleCrit() {
		switch(cornerIndexNumber)
		{
		case 0: 
			if (this.pose.getHeading()>= -1*Math.PI && this.pose.getHeading()<=2*Math.PI) return true;
			break; 
		case 1: 
			if(this.pose.getHeading()>= 0.4*Math.PI && this.pose.getHeading()<=0.6*Math.PI) return true;
			break; 
		case 2:
			if(this.pose.getHeading()>= 0.9*Math.PI && this.pose.getHeading()<=1.1*Math.PI) return true;
			break;
		case 3: 
			if(this.pose.getHeading()>= 1.4*Math.PI && this.pose.getHeading()<=1.6*Math.PI) return true;
			break;
		case 4:
			if(this.pose.getHeading()>= 0.9*Math.PI && this.pose.getHeading()<=1.1*Math.PI) return true;
			break;
		case 5:
			if(this.pose.getHeading()>= 0.4*Math.PI && this.pose.getHeading()<=0.6*Math.PI) return true;
			break;
		case 6:
			if(this.pose.getHeading()>= 0.9*Math.PI && this.pose.getHeading()<=1.1*Math.PI) return true;
			break;
		case 7:
			if(this.pose.getHeading()>= 1.4*Math.PI && this.pose.getHeading()<=1.6*Math.PI) return true;
			break;
		default: return false;
		}
		return false;
	}
	
	public synchronized int getLastCornerNumber() {
		return lastCornerNumber;
	}
	
	public synchronized int getNextCornerNumber() {
		return nextCornerNumber;
	}
	
	public synchronized int getCornerIndex() {
		return cornerIndexNumber;
	}
	
	public synchronized boolean getCornerType() {
		return false;
	}
	
	public synchronized boolean getCornerArea() {
		boolean area = true;
		
		if (mapModus == 1) {
			if ((this.pose.getX()>=0.10)&&(this.pose.getX()<=1.7)&&(this.pose.getY()<=0.10)) {	
				fastAreaIndex = 0;
				area = false;
			}
			/*
			if ((this.pose.getX()>=1.7)&&(this.pose.getY()>=0.10)&&(this.pose.getY()<=0.45)) {
				fastAreaIndex = 1;
				area = false;
			}
			*/
			if ((this.pose.getX()>=0.5)&&(this.pose.getX()<=1.3)&&(this.pose.getY()>=0.20)&&(this.pose.getY()<=0.40)) {
				fastAreaIndex = 2;
				area = false;
			}
			
			if ((this.pose.getX()>=-0.1)&&(this.pose.getX()<=0.1)&&(this.pose.getY()>=0.20)&&(this.pose.getY()<=0.50)) {
				fastAreaIndex = 3;
				area = false;
			}
		}
		return area;
	}
	
	// Private methods
	
	/**
	 * Checks whether there is a back Boundary or not!
	 * @return true if there is a back Boundary
	 */
	private boolean detectBackBoundary() {
		boolean backBoundary = false;
		if (verticalSlot==false) {
			if (perception.getFrontSideSensorDistance() > 28) backBoundary = true;
		}
		else {
			if (perception.getBackSideSensorDistance() > 28) backBoundary = true;
		}
		return backBoundary;
	}
	
	/**
	 * Checks whether there is a front Boundary or not!
	 * @return true if there is a back Boundary
	 */
	private boolean detectFrontBoundary() {
		boolean frontBoundary = false;
		if (perception.getFrontSideSensorDistance() < 17) {
			frontBoundary = true;
		}
		return frontBoundary;
	}
	
	/**
	 * Checks whether the current parking slot is already existing in the database!
	 * @return true --> exists already
	 */
	private boolean checkDublicate() {
		double x_new_Back = newBackBoundaryPosition.getX();
		double y_new_Back = newBackBoundaryPosition.getY();
		double x_new_Front = newFrontBoundaryPosition.getX();
		double y_new_Front = newFrontBoundaryPosition.getY();
		if (slotList.size()>0) {
			for(int i =0; i<slotList.size() ;i++) {
				
				INavigation.ParkingSlot slot = slotList.get(i);
				double x_Back = slot.getFrontBoundaryPosition().getX();	 
				double y_Back = slot.getFrontBoundaryPosition().getY();
				double x_Front = slot.getBackBoundaryPosition().getX();
				double y_Front = slot.getBackBoundaryPosition().getY();
				slotListIndex =  i;
				
				if ((x_new_Back < 1.70)) {										//((y_new_Back < 0.1) ||(y_new_Back>0.2)) && 		// check x-values if the detected slot is horizontal
					if (Math.abs(x_new_Back - x_Back)< 0.05) return true;			
					if (Math.abs(x_new_Front - x_Front)< 0.05) return true;
				}
				
				if ((x_new_Back > 1.70)) {																		// check y-values if the detected slot is vertical
					if ((Math.abs(x_new_Front - x_Front)< 0.05) && (Math.abs(y_new_Front - y_Front)< 0.05)) {
						numberOfSlots++;
						return true;			
					}
				}
			}
		}
		else return false;
		
		return false;
	}
	
	private boolean checkSuitability() {
		double frontValue = Math.abs(this.newFrontBoundaryPosition.getX())+Math.abs(this.newFrontBoundaryPosition.getY());
		double backValue = Math.abs(this.newBackBoundaryPosition.getX())+Math.abs(this.newBackBoundaryPosition.getY());
		
		double parkingSlotLength = Math.abs(frontValue-backValue);
		
		if (parkingSlotLength>0.30) return true;
		else return false;
	}
	
	/**
	 * Adds new parking slots to the array "Slots"
	 */
	private void addParkingSlot() {
		ParkingSlotStatus status= ParkingSlotStatus.SUITABLE_FOR_PARKING;
		if (checkSuitability() == false) status = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
		
		INavigation.ParkingSlot slot = new ParkingSlot(ID,newFrontBoundaryPosition,newBackBoundaryPosition,status,slotMeasurementQuality);
		slotList.add(slot);
		Slots = new ParkingSlot[slotList.size()];
		for (int i=0;i<slotList.size();i++) {
			Slots[i] = slotList.get(i);
		}
		ID++;
	}
	
	/**
	 * overwrites a parking slot if one of its boundaries is already known to the roboter
	 */
	private void overwriteParkingSlot () {
		ParkingSlotStatus status= ParkingSlotStatus.SUITABLE_FOR_PARKING;
		if (checkSuitability() == false) status = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
		
		INavigation.ParkingSlot slot = new ParkingSlot((slotListIndex+1),newFrontBoundaryPosition,newBackBoundaryPosition,status,slotMeasurementQuality);
		slotList.set(slotListIndex,slot);
		Slots = new ParkingSlot[slotList.size()];
		for (int i=0;i<slotList.size();i++) {
			Slots[i] = slotList.get(i);
		}
	}
	
	/**
	*Checks whether the robot is allowed to search for parking slots
	*/
	private boolean checkParkingSlotArea() {
		if (mapModus == 1) {
			if ((this.pose.getX()>=0.05) && (this.pose.getX()<=1.75) && (this.pose.getY()<0.1) ) {	
				verticalSlot = false;
				parkingSlotAreaNumber = 1;
				return true;
			}
	
			if ( (this.pose.getX()>=1.75) && (this.pose.getY()<=0.3) && (Math.abs(this.pose.getHeading()-0.5*Math.PI)<=0.25*Math.PI) ) {
				verticalSlot = true;
				parkingSlotAreaNumber = 2;
				return true;
			}
			
			if ((this.pose.getX()>=1.70) && (this.pose.getY()>0.3) && (Math.abs(this.pose.getHeading()-0.5*Math.PI)<=0.35*Math.PI) ) {
				verticalSlot = true;
				parkingSlotAreaNumber = 3;
				return true;
			}
			
			if ((this.pose.getY()>=0.20) && (this.pose.getY()<=0.40) && (Math.abs(this.pose.getHeading()-Math.PI)<=0.15*Math.PI)) {
				verticalSlot = false;
				parkingSlotAreaNumber = 4;
				return true;
			}
			else {
				foundBackBoundary = false;
				return false;
			}
		}
		else {
			//numberOfSlots=0;
			return false;
		}
	}
	
	/**
	 * calculates an offset for the measurement correction in the parking slot detection depending on the parking slot area 
	 * @return offset in meter as a double
	 */
	private float offsetCorrection() {
		switch(parkingSlotAreaNumber){
			case 1: 
				return 0.03f; 
			case 2: 
				return -0.05f;
			case 3: 
				return 0.05f;
			case 4:
				return -0.07f;
		}
		return 0;
	}
	
	/**
	 * saves the Back-Boundary of a parking slot, distinguishes between the vertical and horizontal slots for the offset correction
	 */
	private void saveBackBoundary () {
		if (verticalSlot==true) {
			newBackBoundaryPosition=new Point( (this.pose.getX()) , (this.pose.getY()+offsetCorrection()) );
		}
		else {
			newBackBoundaryPosition=new Point( (this.pose.getX()+offsetCorrection()) , (this.pose.getY()) );
		}
	}
	
	/**
	 * saves the Front-Boundary of a parking slot, distinguishes between the vertical and horizontal slots for the offset correction
	 */
	private void saveFrontBoundary () {
		if (verticalSlot==true) {
			newFrontBoundaryPosition=new Point( (this.pose.getX()) , (this.pose.getY()+offsetCorrection()) );
		}
		else {
			newFrontBoundaryPosition=new Point( (this.pose.getX()+offsetCorrection()) , (this.pose.getY()) );
		}
	}
	
	/**
	*define the cornerIndexNumber for current pose
	*/
	private void checkAreaIndex() {
		
		if (mapModus == 1) {
			if ((this.pose.getX()<=0.10)&&(this.pose.getY()<=0.10) && cornerIndexNumber != 1) {								
				cornerIndexNumber = 0;
			}
	
			if ((this.pose.getX()>=1.60)&&(this.pose.getY()<=0.15) && cornerIndexNumber != 2) {
				cornerIndexNumber = 1;
			}
		
			if ((this.pose.getX()>=1.60)&&(this.pose.getY()>=0.20) && cornerIndexNumber != 3) {
				cornerIndexNumber = 2;
			}
		
			if ((this.pose.getX()>=1.40)&&(this.pose.getX()<=1.60)&&(this.pose.getY()>=0.50) && cornerIndexNumber != 4) {
				cornerIndexNumber = 3;
			}
		
			if ((this.pose.getX()>=1.40)&&(this.pose.getX()<=1.60)&&(this.pose.getY()>=0.20)&&(this.pose.getY()<=0.50) && cornerIndexNumber != 5) {
				cornerIndexNumber = 4;
			}
		
			if ((this.pose.getX()>=0.20)&&(this.pose.getX()<=0.40)&&(this.pose.getY()>=0.20)&&(this.pose.getY()<=0.50) && cornerIndexNumber != 6) {
				cornerIndexNumber = 5;
			}
		
			if ((this.pose.getX()>=0.20)&&(this.pose.getX()<=0.40)&&(this.pose.getY()>=0.50) && cornerIndexNumber != 7) {
				cornerIndexNumber = 6;
			}
		
			if ((this.pose.getX()<=0.20)&&(this.pose.getY()>=0.50) && cornerIndexNumber != 0) {
				cornerIndexNumber = 7;
			}
		}
		
	}
	/**
	*reset the Angle on the beginning of each fast section
	*/ 
	private void checkAngel() {
		
		
		if(lastcheck != fastAreaIndex){
		
			//long straight
		    if (fastAreaIndex == 0) {	
				this.pose.setHeading((float) (0));
				lastcheck = 0;
				lastCornerNumber = 0;
				nextCornerNumber = 1;
			}
	
		    //shortest straight
			if (fastAreaIndex == 1) {
				this.pose.setHeading((float) (0.50*Math.PI));
				lastcheck = 1;
				lastCornerNumber = 1;
				nextCornerNumber = 2;
			}
			
			//short straight
			if (fastAreaIndex == 2) {
				this.pose.setHeading((float)(Math.PI));
				lastcheck = 2;
				lastCornerNumber = 4;
				nextCornerNumber = 5;
			}
			
			//shortest straight
			if (fastAreaIndex == 3) {
				this.pose.setHeading((float) (1.50*Math.PI));
				lastcheck = 3;
				lastCornerNumber = 7;
				nextCornerNumber = 0;
			}
		
		}
	}
	
	
	
	/**
	 *  evaluates the corner and overwrites the pose if the measured values are meaningful
	 */
	private void evaluateCornerDetection() {
		if (mapModus==1) {	
			switch(lastCornerNumber)
				{
				case 0: 
						//this.pose.setLocation((float)0.00,(float)0.00);
						nextCornerNumber = 1;
					break; 
				case 1: 
						//this.pose.setLocation((float)1.80,(float)0.00);
						nextCornerNumber = 2;
						foundBackBoundary = false;
					break; 
				case 2:
						//this.pose.setLocation((float)1.80,(float)0.60);	
						nextCornerNumber = 3;
						//foundBackBoundary = false;
					break; 
				case 3: 
						this.pose.setLocation((float)1.50,(float)0.60);
						nextCornerNumber = 4;
					break;
				case 4:
						this.pose.setLocation((float)1.50,(float)0.30);
						nextCornerNumber = 5;
					break;
				case 5:
						this.pose.setLocation((float)0.30,(float)0.30);
						nextCornerNumber = 6;
						foundBackBoundary = false;
					break;
				case 6:
						this.pose.setLocation((float)0.30,(float)0.60);
						nextCornerNumber = 7;
					break;
				case 7:
						this.pose.setLocation((float)0.00,(float)0.60);
						nextCornerNumber = 0;
					break;
				default:
				}
		}
		if (mapModus==2) {
			switch(this.getLastCornerNumber())
			{
				case 0: 
						this.pose.setLocation((float)0.00,(float)0.00);
					break; 
				case 1:
						this.pose.setLocation((float)0.75,(float)0.00);
					break; 
				case 2:
						this.pose.setLocation((float)0.75,(float)0.60);	
					break; 
				case 3: 
						this.pose.setLocation((float)0.45,(float)0.60);	
					break;
				case 4:
						this.pose.setLocation((float)0.45,(float)0.30);	
					break;
				case 5:
						this.pose.setLocation((float)0.00,(float)0.30);		
					break;
				default:
			}
		}
	}
	
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.rightLightSensorValue		= perception.getRightLineSensorValue();
		this.leftLightSensorValue 		= perception.getLeftLineSensorValue();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		 	
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation(){
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
		
		double ICCx 		= 0;
		double ICCy 		= 0;

		double xResult 		= 0;
		double yResult 		= 0;
		double angleResult 	= 0;
		
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		if (R.isNaN()) { //robot don't move
			xResult			= this.pose.getX();
			yResult			= this.pose.getY();
			angleResult 	= this.pose.getHeading();
		} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight
			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			angleResult 	= this.pose.getHeading();
		} else {			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult 	= this.pose.getHeading() + w * deltaT;
		}
		
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Lokalisierungsauswertung
		
		if(angleResult>=2*Math.PI) {
			angleResult= angleResult-2*Math.PI;
			//xResult=xResult+0.09;
		}

		if((CurrentStatus.ELSE==Guidance.currentStatus) && (mapModus==1)){
			
			//	Linie 1
			if(this.pose.getX()>0.10&&this.pose.getX()<1.70&&this.pose.getY()<0.10) { 
				
				yDiff=yResult-0;  																		//Abweichung korriegieren
				if(yDiff>=0.01) {                                             
				yResult=0;    	   
					this.pose.setLocation((float)xResult,(float)yResult);                         
					this.pose.setHeading((float)angleResult);   
				} 	
				if(yDiff<=-0.01) {                               
					yResult=0;
					this.pose.setLocation((float)xResult,(float)yResult);                          
					this.pose.setHeading((float)angleResult);
				}
				
				else {                                                									//direkt ausgeben
					this.pose.setLocation((float)xResult,(float)yResult);                            
					this.pose.setHeading((float)angleResult);
				}
			}
			// Linie 2	
			if(this.pose.getY()>0.10&&this.pose.getY()<0.50&&this.pose.getX()>1.70&&this.pose.getX()<1.90){
				xDiff=xResult-1.8;
																										//Abweichung korriegieren
				if(xDiff<=-0.005) { 	   																
					xResult=1.8; 	   		
					this.pose.setLocation((float)xResult,(float)yResult);                         
					this.pose.setHeading((float)angleResult);
				}
				if(xDiff>=0.005) {
					xResult=1.8;
					this.pose.setLocation((float)xResult,(float)yResult);                         
					this.pose.setHeading((float)angleResult);
				}
				else {                                                									//direkt ausgeben
					this.pose.setLocation((float)xResult,(float)yResult);                            
					this.pose.setHeading((float)angleResult);
				}
			}
			//Linie 5
			if(this.pose.getX()>0.40&&this.pose.getX()<1.40&&this.pose.getY()>0.20&&this.pose.getY()<0.4) {				
				yDiff=yResult-0.3;                               																						//Abweichung korriegieren
		    	if(yDiff<=-0.005) {
		    		yResult=0.3;
			   	    this.pose.setLocation((float)xResult,(float)yResult);                          
			       	this.pose.setHeading((float)angleResult);	
			   	}
		    	if(yDiff>=0.005) {
		    		yResult=0.3;	
			   	    this.pose.setLocation((float)xResult,(float)yResult);                          
			       	this.pose.setHeading((float)angleResult);
		    	}
		    	else {                                                									//direkt ausgeben
		    	this.pose.setLocation((float)xResult,(float)yResult);                            
		    	this.pose.setHeading((float)angleResult); 	
				}
			}
			//Linie 8
			   if(this.pose.getX()<0.1&&this.pose.getX()>-0.1&&this.pose.getY()>0.10&&this.pose.getY()<0.50) {//
			   	   xDiff=xResult-0;
				   if(xDiff>=0.01) {                                    								//Abweichung korriegieren
					   xResult=0;
					   this.pose.setLocation((float)xResult,(float)yResult);                         
					   this.pose.setHeading((float)angleResult);
				   }
				   	
				   if(xDiff<=-0.01) {
					   xResult=0;
					   this.pose.setLocation((float)xResult,(float)yResult);                         
					   this.pose.setHeading((float)angleResult);
				   }
			       																						//direkt ausgeben
				   this.pose.setLocation((float)xResult,(float)yResult);                            
			       this.pose.setHeading((float)angleResult);
			   }
		}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);	
	}
		
	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		if (this.checkParkingSlotArea() == true) {
			if ( (foundBackBoundary == false) && (detectBackBoundary() == true)) {
				foundBackBoundary = true;
				this.saveBackBoundary();
			}
			if ( (foundBackBoundary == true) && (detectFrontBoundary() == true)) {
				foundBackBoundary = false;
				this.saveFrontBoundary();
				if (checkDublicate() == false) this.addParkingSlot();
				else this.overwriteParkingSlot();
			}
		}
		return;
	}
}
