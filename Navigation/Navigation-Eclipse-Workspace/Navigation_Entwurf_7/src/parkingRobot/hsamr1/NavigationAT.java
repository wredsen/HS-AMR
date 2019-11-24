package parkingRobot.hsamr1;

import lejos.geom.Line;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.hsamr1.NavigationThread;
import parkingRobot.hsamr1.GuidanceAT;
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
	
	int mapModus = 1; 	// 1 for real, big map and 2 for small test-map
	
	////////////////////////////////////////////////////////////	Variabeln für Lokalisierungsauswertung
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
	 * holds the constant amount of light sensor values in the Linked Lists
	 */
	private static int amountOfValues = 2;
	
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
	private int cornerNumber = 0;
	
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
		
		
		monitor.addNavigationVar("testvariable1");
		monitor.addNavigationVar("testvariable2");
		//monitor.addNavigationVar("cornerNumber");
		
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
		
		if (rightLightSensorList.size()>amountOfValues) {					// deletes old, unnecessary value from lists
			rightLightSensorList.remove(0);
			leftLightSensorList.remove(0);
		}
		
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		if (rightLightSensorList.size() >= 2) {
		testvariable1 = rightLightSensorList.get(1)-rightLightSensorList.get(0);
		testvariable2 = leftLightSensorList.get(1)-leftLightSensorList.get(0);
		}
		
		monitor.writeNavigationVar("testvariable1", "" + testvariable1);			// Monitor Output
		monitor.writeNavigationVar("testvariable2", "" + testvariable2);
		//monitor.writeNavigationVar("cornerNumber", "" + cornerNumber);
		
		LCD.clear();	
		LCD.drawString(" R Diff =" + testvariable1, 0, 0);							// Screen Output
		LCD.drawString(" L Diff =" + testvariable2, 0, 1);
		//LCD.drawString("C.Numm. =" + cornerNumber, 0, 2);
		LCD.drawString("Lichtsen.R =" + rightLightSensorValue, 0, 3);
		LCD.drawString("Lichtsen.L =" + leftLightSensorValue, 0, 4);
		
		this.calculateLocation();
		
		if (getCorner()==true) {						// zum ersten Testen noch nicht notwendig, erst wenn Kurve richtig erkannt und Robo stehen geblieben ist!
			evaluateCornerDetection();
			}
		
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
			return detectCorner();
		}
		else {
			return false;
		}
	}
	
	public synchronized boolean getLeftCorner() {
		return leftCorner;
	}
	
	public synchronized boolean getRightCorner() {
		return rightCorner;
	}
	
	
	public synchronized boolean getCornerArea() {
		boolean safety = false;
		rightCorner = false;
		leftCorner = false;
		
		if (mapModus == 1) {
			if ((this.pose.getX()<=0.10)&&(this.pose.getY()<=0.10)) {								
				safety = true;
				leftCorner = true;
			}
	
			if ((this.pose.getX()>=0.70)&&(this.pose.getY()<=0.10)) {
				safety = true;
				leftCorner = true;
			}
		
			if ((this.pose.getX()>=1.70)&&(this.pose.getY()>=0.50)) {
				safety = true;
				leftCorner = true;
			}
		
			if ((this.pose.getX()>=1.40)&&(this.pose.getX()<=1.60)&&(this.pose.getY()>=0.50)) {
				safety = true;
				leftCorner = true;
			}
		
			if ((this.pose.getX()>=1.40)&&(this.pose.getX()<=1.60)&&(this.pose.getY()>=0.2)&&(this.pose.getY()<=0.4)) {
				safety = true;
				rightCorner = true;
			}
		
			if ((this.pose.getX()>=0.20)&&(this.pose.getX()<=0.40)&&(this.pose.getY()>=0.2)&&(this.pose.getY()<=0.4)) {
				safety = true;
				rightCorner = true;
			}
		
			if ((this.pose.getX()>=0.20)&&(this.pose.getX()<=0.40)&&(this.pose.getY()>=0.50)) {
				safety = true;
				leftCorner = true;
			}
		
			if ((this.pose.getX()<=0.1)&&(this.pose.getY()>=0.50)) {
				safety = true;
				leftCorner = true;
			}
		}
		
		if (mapModus == 2) {
			if ((this.pose.getX()<=0.15)&&(this.pose.getY()<=0.15)) {								
				safety = true;
				leftCorner = true;
			}
	
			if ((this.pose.getX()>=0.60)&&(this.pose.getY()<=0.10)) {
				safety = true;
				leftCorner = true;
			}
		
			if ((this.pose.getX()>=0.60)&&(this.pose.getY()>=0.45)) {
				safety = true;
				leftCorner = true;
			}
		
			if ((this.pose.getX()>=0.35)&&(this.pose.getX()<=0.60)&&(this.pose.getY()>=0.45)) {
				safety = true;
				leftCorner = true;
			}
		
			if ((this.pose.getX()>=0.35)&&(this.pose.getX()<=0.60)&&(this.pose.getY()>=0.2)&&(this.pose.getY()<=0.45)) {
				safety = true;
				rightCorner = true;
			}
		
			if ((this.pose.getX()<=0.15)&&(this.pose.getY()<=0.20)&&(this.pose.getY()<=0.45)) {
				safety = true;
				leftCorner = true;
			}
		}
		
		return safety;
	}
	
	// Private methods	
	
	/**
	 * detects corner, determines type of corner and returns boolean value: corner detected --> true ; no corner detected --> false
	 */
	public boolean detectCorner() {
		boolean corner = false;
		rightCorner = false;
		leftCorner = false;
		if (rightLightSensorList.size()>=2) {
			if (((rightLightSensorList.get(1)-rightLightSensorList.get(0)>=70)||(leftLightSensorList.get(1)-leftLightSensorList.get(0)>=70))) {
				corner = true;
				Sound.twoBeeps();
				cornerNumber++;
			}	
		}
		return corner;	
	}
	
	
	/**
	 *  evaluates the corner and overwrites the pose if the measured values are meaningful
	 */
	private void evaluateCornerDetection() {
			switch(cornerNumber)
			{
				case '0': 
					if (leftCorner=true) {
						this.pose.setLocation((float)0.00,(float)0.00);
					}
					break; 
				case '1': 
					if (leftCorner=true) {
						this.pose.setLocation((float)1.80,(float)0.00);
					}
					break; 
				case '2':
					if(leftCorner=true) {
						this.pose.setLocation((float)1.80,(float)0.60);	
					}
					break; 
				case '3': 
					if(leftCorner=true) {
						this.pose.setLocation((float)1.50,(float)0.60);	
					}
					break;
				case '4':
					if(rightCorner=true) {
						this.pose.setLocation((float)1.50,(float)0.30);	
					}
					break;
				case '5':
					if(rightCorner=true) {
						this.pose.setLocation((float)0.30,(float)0.30);	
					}
					break;
				case '6':
					if(leftCorner=true) {
						this.pose.setLocation((float)0.30,(float)0.60);	
					}
					break;
				case '7':
					if(leftCorner=true) {
						this.pose.setLocation((float)0.00,(float)0.60);	
					}
					break;
				default:
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
				
				if(!(GuidanceAT.getCurrentStatus() == GuidanceAT.CurrentStatus.INACTIVE || 
				   GuidanceAT.getCurrentStatus() == GuidanceAT.CurrentStatus.EXIT)){
				//	Linie 1
					if(this.pose.getX()>=0.05&&this.pose.getX()<=1.75&&this.pose.getY()<0.10) { 
				    	          
						    angleDiff=angleResult-0;           //Abweichung korriegieren ohne Einparken bzw Einparken?
							if(angleDiff>0.1*Math.PI) {
					    		   angleResult=0;
					    	}
							
							if(angleDiff<-0.05*Math.PI) {
					    		   angleResult=0;
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
				// Linie 2	
					if(angleResult>0.3*Math.PI&&angleResult<0.9*Math.PI&&this.pose.getX()>=1.75&&this.pose.getX()<=1.83){
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
				// Linie 5
					if(angleResult>0.9*Math.PI&&angleResult<1.1*Math.PI&&this.pose.getX()>=0.3&&this.pose.getX()<1.47&&this.pose.getY()<0.4) {
						yDiff=yResult-0.3;                               
																												//Abweichung korriegieren
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
				// Linie 8
					   if(this.pose.getX()<=0.1&&this.pose.getX()>=-0.1&&angleResult>=1.4*Math.PI&&angleResult<=1.6*Math.PI) {//
						   
					   	   angleDiff=angleResult-1.5*Math.PI;            										//Angle Abweichung korriegieren                  
					   	   if(angleDiff>0.1*Math.PI) {
					   		   angleResult=1.5*Math.PI;
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
		return; // has to be implemented by students
	}
}
