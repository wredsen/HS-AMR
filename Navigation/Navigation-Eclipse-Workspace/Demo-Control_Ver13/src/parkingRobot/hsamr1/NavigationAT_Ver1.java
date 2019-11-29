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
import parkingRobot.hsamr1.NavigationThread_Ver1;
import parkingRobot.hsamr1.Guidance_Ver1;
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
public class NavigationAT_Ver1 implements INavigation{
	
	////////////////////////////////////////////////////////////	Variabeln fuer Parklueckenerkennung
	boolean findslotin1=false;
	double xRealeFrontPositionParkplatz=0;
	double yRealeFrontPositionParkplatz=0;
	double xRealeBackPositionParkplatz=0;
	double yRealeBackPositionParkplatz=0;
	
	int parkID=0;                                                       //Startwerte fur ParklueckenDetektion
	ArrayList<ParkingSlot>slotList=new ArrayList<ParkingSlot>();       //private Methode
	Point backBoundaryPositionA=null;
	Point backBoundaryPositionB=null;
	Point frontBoundaryPositionA=null;
	Point frontBoundaryPositionB=null;
	Point backBoundaryPosition=null;
	Point frontBoundaryPosition=null;
	
	boolean Long=false;
	boolean EinparkenSignal=false;
	
	int measurementQuality=0;                                                //measurementQuality  Variable
	double BackboundaryZeit=0;                                          //Variablen fuer QualitaetsBewertung
	double FrontboundaryZeit=0;
	double Ecke1Zeit=0;
	double Ecke2Zeit=0;
	double Ecke4Zeit=0;
	double Ecke5Zeit=0;
	double tSum=0;
	
	double Gut=0;
	double Bad=0;
	
	double frontpark=0;
	double backpark=0;
	
	double parkingslotlong=0;
	INavigation.ParkingSlot[] Slots=null;  
	////////////////////////////////////////////////////////////
	int mapModus = 1; 	// 1 for real, big map and 2 for small test-map
	
	////////////////////////////////////////////////////////////	Variabeln fuer Lokalisierungsauswertung
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
	double oldFirstValue = -3;
	
	/**
	 * holds the value for the average difference of the sensor that DOES go over the black line in a Corner
	 */
	
	double oldSecondValue = 0;
	
	/**
	 * holds the constant amount of light sensor values in the Linked Lists
	 */
	private static int amountOfValues = 100;
	
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
	 * holds the index number of the current/last corner
	 */
	private int cornerNumber = 0;
	
	/**
	 * holds the index number of the current/last corner area
	 */
	private int areaNumber = 0;
	
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
	NavigationThread_Ver1 navThread = new NavigationThread_Ver1(this);

	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT_Ver1(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();		
		
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
		
		monitor.addNavigationVar("cornerNumber");
		monitor.addNavigationVar("X");
		monitor.addNavigationVar("Y");
		monitor.addNavigationVar("angle");
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
		
		angleList.add(this.pose.getHeading());		// adds the angle Value to the list
		
		if (angleList.size()>amountOfValues) {					// deletes old, unnecessary value from list
			angleList.remove(0);
		}
		
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();		
		this.calculateLocation();
		this.getCorner();
		
		LCD.clear();
		LCD.drawString("Area = "+ this.getCornerArea(), 0, 1);
		LCD.drawString("Cor Num = "+ this.getCornerNumber(), 0, 2);
		LCD.drawString("X = "+ this.pose.getX(), 0, 3);
		LCD.drawString("Y = "+ this.pose.getY(), 0, 4);
		
		monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
		monitor.writeNavigationVar("X", "" + this.pose.getX());
		monitor.writeNavigationVar("Y", "" + this.pose.getY());
		monitor.writeNavigationVar("angle", "" + this.pose.getHeading());
		
			
		//monitor.writeNavigationVar("areaNumber", "" + this.areaNumber);	
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
	
	
	public synchronized boolean getCorner() {
		if (getCornerArea() == true) {
			if (detectCorner()) {
				evaluateCornerDetection();
				return true;
			}
			else return false;
		}
		else return false;
	}
	
	public synchronized int getCornerNumber() {
		return cornerNumber;
	}
	
	public synchronized boolean getCornerType() {
		return false;
	}
	
	public synchronized boolean getCornerArea() {
		boolean area = false;
		
		if (mapModus == 1) {
			if ((this.pose.getX()<=0.15)&&(this.pose.getY()<=0.15)) {								
				area = true;
			}
	
			if ((this.pose.getX()>=1.65)&&(this.pose.getY()<=0.15)) {
				area = true;
			}
		
			if ((this.pose.getX()>=1.65)&&(this.pose.getY()>=0.40)) {
				area = true;
			}
		
			if ((this.pose.getX()>=1.30)&&(this.pose.getX()<=1.65)&&(this.pose.getY()>=0.40)) {
				area = true;
			}
		
			if ((this.pose.getX()>=1.35)&&(this.pose.getX()<=1.65)&&(this.pose.getY()>=0.20)&&(this.pose.getY()<=0.45)) {
				area = true;
			}
		
			if ((this.pose.getX()>=0.20)&&(this.pose.getX()<=0.40)&&(this.pose.getY()>=0.20)&&(this.pose.getY()<=0.45)) {
				area = true;
			}
		
			if ((this.pose.getX()>=0.20)&&(this.pose.getX()<=0.40)&&(this.pose.getY()>=0.45)) {
				area = true;
			}
		
			if ((this.pose.getX()<=0.15)&&(this.pose.getY()>=0.45)) {
				area = true;
			}
		}
		
		if (mapModus == 2) {
			if ((this.pose.getX()<=0.15)&&(this.pose.getY()<=0.15)) {								
				area = true;
			}
	
			if ((this.pose.getX()>=0.60)&&(this.pose.getY()<=0.10)) {
				area = true;
			}
		
			if ((this.pose.getX()>=0.67)&&(this.pose.getY()>=0.45)) {
				area = true;
			}
		
			if ((this.pose.getX()>=0.35)&&(this.pose.getX()<=0.60)&&(this.pose.getY()>=0.45)) {
				area = true;
			}
		
			if ((this.pose.getX()>=0.35)&&(this.pose.getX()<=0.60)&&(this.pose.getY()>=0.2)&&(this.pose.getY()<=0.45)) {
				area = true;
			}
		
			if ((this.pose.getX()<=0.15)&&(this.pose.getY()<=0.20)&&(this.pose.getY()<=0.45)) {
				area = true;
			}
		}
		
		return area;
	}
	
	// Private methods	
	
	/**
	 * detects corner, determines type of corner and returns boolean value: corner detected --> true ; no corner detected --> false
	 */
	public boolean detectCorner() {
		boolean corner = false;
		if (this.angleList.size()>=amountOfValues) {
			double newFirstValue = angleList.get(0);
			double newSecondValue = angleList.get((amountOfValues-1));
			if (Math.abs(newFirstValue - oldFirstValue) >= 0.3*Math.PI) {
				if (Math.abs(newSecondValue - oldFirstValue) >=0.3*Math.PI) {
					if (Math.abs(newSecondValue-newFirstValue) >= 0.4*Math.PI) {
					oldFirstValue = newFirstValue;
					oldSecondValue = newSecondValue;
					cornerNumber++;
					cornerNumber = cornerNumber%8;
					corner = true;
					}
				}
			}	
		}
		return corner;
	}
	
	
	/**
	 *  evaluates the corner and overwrites the pose if the measured values are meaningful
	 */
	private void evaluateCornerDetection() {
		
		if (mapModus==1) {	
			switch(cornerNumber)
				{
				case 0: 
						this.pose.setLocation((float)0.00,(float)0.00);
						this.pose.setHeading((float)0.00);
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break; 
				case 1: 
						this.pose.setLocation((float)1.80,(float)0.00);
						this.pose.setHeading((float) (0.50*Math.PI));
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break; 
				case 2:
						this.pose.setLocation((float)1.80,(float)0.60);
						this.pose.setHeading((float) (Math.PI));
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break; 
				case 3: 
						this.pose.setLocation((float)1.50,(float)0.60);
						this.pose.setHeading((float) (1.50*Math.PI));
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break;
				case 4:
						this.pose.setLocation((float)1.50,(float)0.30);
						this.pose.setHeading((float) (Math.PI));
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break;
				case 5:
						this.pose.setLocation((float)0.30,(float)0.30);
						this.pose.setHeading((float) (0.50*Math.PI));
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break;
				case 6:
						this.pose.setLocation((float)0.30,(float)0.60);
						this.pose.setHeading((float) (Math.PI));
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break;
				case 7:
						this.pose.setLocation((float)0.00,(float)0.60);
						this.pose.setHeading((float) (1.50*Math.PI));
						//monitor.writeNavigationVar("cornerNumber", "" + this.cornerNumber);
						//monitor.writeNavigationVar("X", "" + this.pose.getX());
						//monitor.writeNavigationVar("Y", "" + this.pose.getY());
					break;
				default:
				}
		}
		if (mapModus==2) {
			switch(this.getCornerNumber())
			{
				case 0: 
						this.pose.setLocation((float)0.00,(float)0.00);
					break; 
				case 1:
						Sound.twoBeeps();
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
			
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	Lokalisierungsauswertung
			
			if(angleResult>=2*Math.PI) {
				 angleResult= angleResult-2*Math.PI;
				 //xResult=xResult+0.09;
				
				}
				
				if(!(Guidance_Ver1.getCurrentStatus() == Guidance_Ver1.CurrentStatus.INACTIVE || 
				   Guidance_Ver1.getCurrentStatus() == Guidance_Ver1.CurrentStatus.EXIT) && (mapModus==1)){
				//	Linie 1
					if(this.pose.getX()>0.15&&this.pose.getX()<1.65&&this.pose.getY()<0.15) { 
				    	          
						    angleDiff=angleResult-0;           //Abweichung korriegieren ohne Einparken bzw Einparken?
							if(angleDiff>0.05*Math.PI) {
					    		   angleResult=0;
					    		   this.pose.setHeading((float)angleResult);
					    	}
							
							if(angleDiff<-0.05*Math.PI) {
					    		   angleResult=0;
					    		   this.pose.setHeading((float)angleResult);
							}
							
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
				
				//Linie 2	
					if(this.pose.getY()>0.15 && this.pose.getY()<0.45 && this.pose.getX()>1.65 && this.pose.getX()<1.90){
					    
						angleDiff=angleResult-0.5*Math.PI;           
						if(angleDiff>0.05*Math.PI) {
				    		   angleResult=0.5*Math.PI;
				    		   this.pose.setHeading((float)angleResult);
				    	}
						
						if(angleDiff<-0.05*Math.PI) {
				    		   angleResult=0.5*Math.PI;
				    		   this.pose.setHeading((float)angleResult);
						}
						
						xDiff=xResult-1.8;																						//Abweichung korriegieren
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
					
				/*	
				// Linie 3
				    if(angleResult>0.9*Math.PI&&angleResult<1.1*Math.PI&&this.pose.getX()>=1.47&&this.pose.getY()>=0.45) {
				    	yDiff=yResult-0.6;
				    																							//Abweichung korriegieren
				    	if(yDiff>=0.01) {
				    		yResult=0.6;
				    	   this.pose.setLocation((float)xResult,(float)yResult);                         
				    	   this.pose.setHeading((float)angleResult);
				    	} 	
				    	if(yDiff<=-0.01) {
				    		yResult=0.6;
					   	    this.pose.setLocation((float)xResult,(float)yResult);                          
					       	this.pose.setHeading((float)angleResult);
					   	}
					    else {                                                									//direkt ausgeben
						    this.pose.setLocation((float)xResult,(float)yResult);                            
						    this.pose.setHeading((float)angleResult);
						}	
				    }				
				// Linie 4
				    if(angleResult>=1.4*Math.PI&&angleResult<1.6*Math.PI&&this.pose.getX()>=1.48&&this.pose.getX()<=1.53) {
				    	xDiff=xResult-1.5;
				    																							//Abweichung korriegieren
				    	if(xDiff>=0.01) {
				    		xResult=1.5;
				    		this.pose.setLocation((float)xResult,(float)yResult);                         
				    	    this.pose.setHeading((float)angleResult);  
				    	}
				    	if(xDiff<=-0.01) {
				    	   	xResult=1.5;
				    	   	this.pose.setLocation((float)xResult,(float)yResult);                         
				    	    this.pose.setHeading((float)angleResult);
				    	}
				    	else {                                                									//direkt ausgeben
				    		this.pose.setLocation((float)xResult,(float)yResult);                            
				    	    this.pose.setHeading((float)angleResult);
				    	}		   
				    }
				    
				*/	
				// Linie 5
					if(this.pose.getX()>0.45 && this.pose.getX()<1.35 && this.pose.getY()>0.20 && this.pose.getY()<0.4) {
						
						angleDiff=angleResult-Math.PI;           
						if(angleDiff>0.05*Math.PI) {
				    		   angleResult=Math.PI;
				    		   this.pose.setHeading((float)angleResult);
				    	}
						
						if(angleDiff<-0.05*Math.PI) {
				    		   angleResult=Math.PI;
				    		   this.pose.setHeading((float)angleResult);
						}
						
						yDiff=yResult-0.3;                               
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
					
				/*	
				// Linie 6
					   if(angleResult>0.4*Math.PI&&angleResult<0.8*Math.PI&&this.pose.getX()>=0.25&&this.pose.getX()<=0.35) {    
							  xDiff=xResult-0.3;
							  																					//Abweichung korriegieren
						   	if(xDiff>=0.01) {
						   		xResult=0.3;
						   		this.pose.setLocation((float)xResult,(float)yResult);                         
						       	this.pose.setHeading((float)angleResult);
						   	}
						   	if(xDiff<=-0.01) {
						   		xResult=0.3;
						   		this.pose.setLocation((float)xResult,(float)yResult);                         
						       	this.pose.setHeading((float)angleResult);
						   	}
						   	else {                                                								//direkt ausgeben
						       	this.pose.setLocation((float)xResult,(float)yResult);                            
						       	this.pose.setHeading((float)angleResult);
						    }		
					   }
				// Linie 7
					   if(angleResult>0.9*Math.PI&&angleResult<1.1*Math.PI&&this.pose.getX()>0.05&&this.pose.getX()<=0.3) {
						   yDiff=yResult-0.6;                               
						   																						//Abweichung korriegieren
						   	if(yDiff>=0.01) {
					   	   		yResult=0.6;
					   	   		this.pose.setLocation((float)xResult,(float)yResult);                         
					   	   		this.pose.setHeading((float)angleResult);
					   		} 	
						   	if(yDiff<=-0.01) {
						   		yResult=0.6;
						   		this.pose.setLocation((float)xResult,(float)yResult);                          
						   		this.pose.setHeading((float)angleResult);	
						   	}
						   	else {                                                								//direkt ausgeben
						   		this.pose.setLocation((float)xResult,(float)yResult);                            
						   		this.pose.setHeading((float)angleResult);
						   	}	 
					   }
				*/	   					   
				// Linie 8
					   if(this.pose.getX()<0.1 && this.pose.getX()>-0.1 && this.pose.getY()>0.15 && this.pose.getY()<0.45) {//
						   
							angleDiff=angleResult-1.5*Math.PI;           
							if(angleDiff>0.05*Math.PI) {
					    		   angleResult=1.5*Math.PI;
					    		   this.pose.setHeading((float)angleResult);
					    	}
							
							if(angleDiff<-0.05*Math.PI) {
					    		   angleResult=1.5*Math.PI;
					    		   this.pose.setHeading((float)angleResult);
							} 	
						       
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
		
		if((this.frontSideSensorDistance>45)&&(findslotin1==false)) {                          //BackboundaryPosition
			
			
			if((this.pose.getY()<0.15)&&(this.pose.getHeading() < 0.25*Math.PI)&&(this.pose.getHeading()> -0.2*Math.PI))    //Parklucken Linie1 
			{
				xRealeBackPositionParkplatz=this.pose.getX()+0.035-0.0565;
				yRealeBackPositionParkplatz=this.pose.getY();
					
				backBoundaryPosition=new Point((float)xRealeBackPositionParkplatz,(float)yRealeBackPositionParkplatz);
				findslotin1=true;
				BackboundaryZeit=tSum;
				Sound.twoBeeps();
			}
			if(this.pose.getX()>1.75 &&(this.pose.getHeading() < 0.6*Math.PI)&&(this.pose.getHeading()> 0.4*Math.PI))     //Parklucken Linie2
			{
				
				xRealeBackPositionParkplatz=this.pose.getX();
				yRealeBackPositionParkplatz=this.pose.getY()+0.035-0.0565;
					
				backBoundaryPosition=new Point((float)xRealeBackPositionParkplatz,(float)yRealeBackPositionParkplatz);
				findslotin1=true;
				BackboundaryZeit=tSum;
				Sound.twoBeeps();
			}
			if((this.pose.getX()<=1.28)&&(this.pose.getX()>=0.45)&&(this.pose.getY()>=0.25)&&(this.pose.getHeading() < 1.1*Math.PI)&&(this.pose.getHeading()> 0.9*Math.PI))     //Parklucken  Linie 5
			{
				
				xRealeBackPositionParkplatz=this.pose.getX()-0.035+0.0565;
				yRealeBackPositionParkplatz=this.pose.getY();	
					
				backBoundaryPosition=new Point((float)xRealeBackPositionParkplatz,(float)yRealeBackPositionParkplatz);
				findslotin1=true;
				BackboundaryZeit=tSum;
				Sound.twoBeeps();
			}
			
		}
		
		if ((findslotin1==true)&&(this.frontSideSensorDistance<15))         //frontBoundaryPosition
		
		{
			if((this.pose.getHeading() < 0.25*Math.PI)&&(this.pose.getHeading()> -0.2*Math.PI)&&(this.pose.getY()<0.15))    //Parklucken Linie1
			{
				xRealeFrontPositionParkplatz=this.pose.getX()+0.035-0.0565;
				yRealeFrontPositionParkplatz=this.pose.getY();
				
				frontBoundaryPosition= new Point((float)xRealeFrontPositionParkplatz,(float)yRealeFrontPositionParkplatz);
				FrontboundaryZeit=tSum;
				findslotin1=false;
				Long=true;	
				measurementQuality=(int)(100-(FrontboundaryZeit+BackboundaryZeit)*100/2*Ecke1Zeit);               //Parklucken measurementQualitat
				Sound.beep();
			}
			if(this.pose.getX()>1.75&&(this.pose.getHeading() < 0.6*Math.PI)&&(this.pose.getHeading()> 0.4*Math.PI))     //Parklucken Linie2
			{
				
				xRealeFrontPositionParkplatz=this.pose.getX();
				yRealeFrontPositionParkplatz=this.pose.getY()+0.035-0.0565;
				
				frontBoundaryPosition= new Point((float)xRealeFrontPositionParkplatz,(float)yRealeFrontPositionParkplatz);
				FrontboundaryZeit=tSum;
				findslotin1=false;
				Long=true;
				measurementQuality=(int)(100-(FrontboundaryZeit+BackboundaryZeit-2*Ecke1Zeit)*100/2*(Ecke2Zeit-Ecke1Zeit));       //Parklucken measurementQualitat
				Sound.beep();
			}
			if((this.pose.getX()<=1.28)&&(this.pose.getX()>=0.45)&&(this.pose.getY()>=0.25)&&(this.pose.getY()>=0.25)&&(this.pose.getHeading() < 1.1*Math.PI)&&(this.pose.getHeading()> 0.9*Math.PI))    //Parklucken  Linie 5
			{
				
				xRealeFrontPositionParkplatz=this.pose.getX()-0.035+0.0565;
				yRealeFrontPositionParkplatz=this.pose.getY();	
				
				frontBoundaryPosition= new Point((float)xRealeFrontPositionParkplatz,(float)yRealeFrontPositionParkplatz);
				FrontboundaryZeit=tSum;
				findslotin1=false;
				Long=true;
				measurementQuality=(int)(100-(FrontboundaryZeit+BackboundaryZeit-2*Ecke4Zeit)*100/2*(Ecke5Zeit-Ecke4Zeit));       //Parklucken measurementQualitat
				Sound.beep();
			}
			}
			
		if(slotList.size() > 0) {
			boolean parkplatzVorhanden = false;
			if(this.pose.getX()>1.75&&(this.pose.getHeading() < 0.6*Math.PI)) 
			{
				for(int i =0; i<slotList.size();i++) {
					ParkingSlot parkingSlot = slotList.get(i);
					double xBackAktuell = this.backBoundaryPosition.getX(); //Speichert aktuell gemessenen X-Wert der Backboundary
					double yBackAktuell = this.backBoundaryPosition.getY(); //Speichert aktuell gemessenen Y-Wert der Backboundary
					double xFrontAktuell = this.frontBoundaryPosition.getX(); //Speichert aktuell gemessenen X-Wert der frontBoundaryPosition
					double yFrontAktuell = this.frontBoundaryPosition.getY(); //Speichert aktuell gemessenen Y-Wert der frontBoundaryPosition
					
					double xBackListe = parkingSlot.getBackBoundaryPosition().getX(); //Speichert den X-Wert vom Parkplatz in der Datenbank
					double yBackListe = parkingSlot.getBackBoundaryPosition().getY(); //Speichert den Y-Wert vom Parkplatz in der Datenbank
					double xFrontListe = parkingSlot.getFrontBoundaryPosition().getX(); //Speichert den X-Wert vom Parkplatz in der Datenbank
					double yFrontListe = parkingSlot.getFrontBoundaryPosition().getY(); //Speichert den Y-Wert vom Parkplatz in der Datenbank
					
					if(((xBackAktuell - 0.02) <= xBackListe) && (xBackListe <= (xBackAktuell + 0.02))){
						//parkplatzVorhanden = true;
					}
					
					if(((yBackAktuell - 0.01) <= yBackListe) && (yBackListe <= (yBackAktuell + 0.01))){
						parkplatzVorhanden = true;
					}
					
					if(((xFrontAktuell - 0.01) <= xFrontListe) && (xFrontListe <= (xFrontAktuell + 0.01))){
						//parkplatzVorhanden = true;
					}
					
					if(((yFrontAktuell - 0.01) <= yFrontListe) && (yFrontListe <= (yFrontAktuell + 0.01))){
						parkplatzVorhanden = true;
					}
					
				}
			}else {
			for(int i =0; i<slotList.size();i++) {
				ParkingSlot parkingSlot = slotList.get(i);
				double xBackAktuell = this.backBoundaryPosition.getX(); //Speichert aktuell gemessenen X-Wert der Backboundary
				double yBackAktuell = this.backBoundaryPosition.getY(); //Speichert aktuell gemessenen Y-Wert der Backboundary
				double xFrontAktuell = this.frontBoundaryPosition.getX(); //Speichert aktuell gemessenen X-Wert der frontBoundaryPosition
				double yFrontAktuell = this.frontBoundaryPosition.getY(); //Speichert aktuell gemessenen Y-Wert der frontBoundaryPosition
				
				double xBackListe = parkingSlot.getBackBoundaryPosition().getX(); //Speichert den X-Wert vom Parkplatz in der Datenbank
				double yBackListe = parkingSlot.getBackBoundaryPosition().getY(); //Speichert den Y-Wert vom Parkplatz in der Datenbank
				double xFrontListe = parkingSlot.getFrontBoundaryPosition().getX(); //Speichert den X-Wert vom Parkplatz in der Datenbank
				double yFrontListe = parkingSlot.getFrontBoundaryPosition().getY(); //Speichert den Y-Wert vom Parkplatz in der Datenbank
				
				if(((xBackAktuell - 0.02) <= xBackListe) && (xBackListe <= (xBackAktuell + 0.02))){
					parkplatzVorhanden = true;
				}
				
				if(((yBackAktuell - 0.02) <= yBackListe) && (yBackListe <= (yBackAktuell + 0.02))){
					//parkplatzVorhanden = true;
				}
				
				if(((xFrontAktuell - 0.02) <= xFrontListe) && (xFrontListe <= (xFrontAktuell + 0.02))){
					parkplatzVorhanden = true;
				}
				
				if(((yFrontAktuell - 0.02) <= yFrontListe) && (yFrontListe <= (yFrontAktuell + 0.02))){
				//	parkplatzVorhanden = true;
				}
			}
				
				if(parkplatzVorhanden == false) {
					parkplatzHinzufuegen();
				}
			}
		}
		else {
			//einfach Parkplatz hinzufuegen
			parkplatzHinzufuegen();
		}
	}
	
	void parkplatzHinzufuegen() {
		
		if(Long==true) {
			
			frontpark=Math.abs(this.frontBoundaryPosition.getX())+Math.abs(this.frontBoundaryPosition.getY());
			backpark=Math.abs(this.backBoundaryPosition.getX())+Math.abs(this.frontBoundaryPosition.getY());
			parkingslotlong=frontpark-backpark;                                //Parkluckenlange
			parkID++;
			
			if(parkingslotlong<0) {
				parkingslotlong=-parkingslotlong;
			}
			if(parkingslotlong>=0.45) {                                       //ParkluckenBewertung
				ParkingSlotStatus status=ParkingSlotStatus.SUITABLE_FOR_PARKING;	
				ParkingSlot parkingSlots=new ParkingSlot(parkID,frontBoundaryPosition,backBoundaryPosition,status,measurementQuality);
				slotList.add(parkingSlots);                                    //Datenbank  
				
				Slots = new ParkingSlot[slotList.size()];
				for (int i=0;i<slotList.size();i++) {
					Slots[i] = slotList.get(i);
				}
			
				Gut=1;
				Bad=0;
			}
			
			if(parkingslotlong<0.45) {                                    //ParkluckenBewertung
				ParkingSlotStatus status=ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
				ParkingSlot parkingSlots=new ParkingSlot(parkID,frontBoundaryPosition,backBoundaryPosition,status,measurementQuality);
				slotList.add(parkingSlots);                                //Datenbank
				
				Slots = new ParkingSlot[slotList.size()];                      
				for (int i=0;i<slotList.size();i++) {
					Slots[i] = slotList.get(i);
				}
			
				Bad=1;
				Gut=0;
			}
				
			Long = false;
		}
	}
}