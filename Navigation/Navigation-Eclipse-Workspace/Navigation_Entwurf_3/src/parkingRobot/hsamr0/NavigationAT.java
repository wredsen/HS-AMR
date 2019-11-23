package parkingRobot.hsamr0;

import lejos.geom.Line;
import lejos.nxt.LCD;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import parkingRobot.hsamr0.NavigationThread;

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


	double testvariable1 = 0;
	double testvariable2 = 0;
	double testvariable3 = 0;
	
	/**
	 * holds the value for the minimal difference the sensors have to detect if there is a corner
	 */
	float minimalDifference =50;
	
	/**
	 * holds the constant amount of light sensor values in the Linked Lists
	 */
	private static int amountOfValues = 10;
	
	/**
	 * Linked List for right light sensor values
	 */
	LinkedList<Integer> rightLightSensorList = new LinkedList<Integer>();
	
	/**
	 * Linked List for left light sensor values
	 */
	LinkedList<Integer> leftLightSensorList = new LinkedList<Integer>();
	
	/**
	 * color information measured by right light sensor: 0% --> black , 100% --> white
	 */
	int rightLightSensorValue = 0;
	
	/**
	 * color information measured by left light sensor: 0% --> black , 100% --> white
	 */
	int leftLightSensorValue = 0;
	
	/**
	 * indicates whether there is a right corner or not
	 */
	boolean rightCorner = false;
	
	/**
	 * indicates whether there is a right corner or not
	 */
	boolean leftCorner = false;
	
	/**
	 * holds the index number of the current/last corner
	 */
	int cornerNumber = 0;
	
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
	static final double WHEEL_DISTANCE		= 	0.114; // only rough guess, to be measured exactly and maybe refined by experiments

	
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
		
		rightLightSensorList.add(rightLightSensorValue);		// adds the current light sensor Values to the lists
		leftLightSensorList.add(leftLightSensorValue);
		
		if (rightLightSensorList.size()>10) {					// deletes old, unnecessary value from lists
			rightLightSensorList.remove(0);
			leftLightSensorList.remove(0);
		}
		
		detectCorner();
		
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		testvariable2 = getAverageDiff(rightLightSensorList);
		testvariable1 = getAverageDiff(leftLightSensorList);
		
		LCD.clear();	
		LCD.drawString("Diff. L =" + testvariable1, 0, 0);
		LCD.drawString("Diff. R =" + testvariable2, 0, 1);
		LCD.drawString("C.Numm. =" + cornerNumber, 0, 2);

		this.calculateLocation();
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
		return null;
	}
	
	
	// Private methods
	
	/**
	 * Calculates the Average Difference of a Linked List to the amountOfValues Point
	 * @return Average Difference
	 */
	private double getAverageDiff (LinkedList<Integer> list) {
		double d = 0;
		double a1 = 0;
		double a2 = 0;
		
		if (list.size() >= 10) {
			for(int i=(list.size()-amountOfValues); i<(list.size()-(amountOfValues/2)); i++){
				a1 += list.get(i);
			}
			a1 = a1/(amountOfValues/2);
			
			for(int i=(list.size()-(amountOfValues/2)); i<list.size(); i++){
				a2 += list.get(i);
			}
			a2 = a2/(amountOfValues/2);
			
			d = a1 - a2;
		}
		else {d=0;}

		return d;
	}
	
	
	/**
	 * detects corner and returns boolean value: corner detected --> true ; no corner detected --> false
	 */
	private int detectCorner() {
		rightCorner = false;
		leftCorner = false;

		if((getAverageDiff(rightLightSensorList)>minimalDifference)&&(getAverageDiff(leftLightSensorList)<minimalDifference)) {
			rightCorner = true;
			cornerNumber++;
		}
		
		if((getAverageDiff(leftLightSensorList)>minimalDifference)&&(getAverageDiff(rightLightSensorList)<minimalDifference)) {
			leftCorner = true;
			cornerNumber++;
		}
		
		cornerNumber = cornerNumber%8;
		
		return cornerNumber;
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
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);		 
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		return; // has to be implemented by students
	}
}