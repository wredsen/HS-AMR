package parkingRobot.hsamr1;

import lejos.geom.Line;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.hsamr1.NavigationThread;
import parkingRobot.hsamr1.GuiDemo2.CurrentStatus;
import parkingRobot.IMonitor;


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
	
	public boolean getCornerType() {
		return true;
	}
	
	public boolean getLeftCorner() {
		return true;
	}
			
	public boolean getRightCorner() {
		return true;
	}
	
	public boolean getCorner() {
		return true;
	}
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
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = map;
	}
	
	public void setPose(Pose pose) {
		this.pose = pose; 
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
		this.calculateLocation();
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		// MONITOR (example)
//		monitor.writeNavigationComment("Navigation");
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
	
	/**
	 * Signals when the robot is in the near of a corner
	 * @return false if the robot is not in the near of a corner, true if it is
	 */
	public synchronized boolean getCornerArea() {
		boolean area = true;
			if ( (this.pose.getX()>=0.10) && (this.pose.getX()<=1.70) && (this.pose.getY()<=0.10) ) area = false;
			if ( (this.pose.getX()>=1.70) && (this.pose.getY()>=0.15) && (this.pose.getY()<=0.50) ) area = false;
			if ( (this.pose.getX()>=0.50) && (this.pose.getX()<=1.30) && (this.pose.getY()>=0.20) && (this.pose.getY()<=0.40) ) area = false;
			if ( (this.pose.getX()>=-0.1) && (this.pose.getX()<=0.10) && (this.pose.getY()>=0.20) && (this.pose.getY()<=0.50) ) area = false;
		return area;
	}
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
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
		
		if(angleResult>=2*Math.PI)	angleResult= angleResult-2*Math.PI;	

		if(CurrentStatus.LINE_FOLLOW==GuiDemo2.currentStatus){
			
			//	Linie 1
			if( this.pose.getX()>0.10 && this.pose.getX()<1.70 && this.pose.getY()<0.04 ) { 
				double yDiff=yResult-0;  																		
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
				else {                                                									
					this.pose.setLocation((float)xResult,(float)yResult);                            
					this.pose.setHeading((float)angleResult);
				}
			}
			
			// Linie 2	
			if( this.pose.getY()>0.10 && this.pose.getY()<0.50 && this.pose.getX()>1.75 && this.pose.getX()<1.85 ){
				double xDiff=xResult-1.8;																						
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
			if( this.pose.getX()>0.40 && this.pose.getX()<1.40 && this.pose.getY()>0.20 && this.pose.getY()<0.35 ) {				
				double yDiff=yResult-0.3;                               																						//Abweichung korriegieren
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
		    	else {                                                									
		    		this.pose.setLocation((float)xResult,(float)yResult);                            
		    		this.pose.setHeading((float)angleResult); 	
				}
			}
			
			//Linie 8
			   if( this.pose.getX()<0.5 && this.pose.getX()>-0.05 && this.pose.getY()>0.10 && this.pose.getY()<0.50 ) {//
			   	   double xDiff=xResult-0;
				   if(xDiff>=0.01) {                                    								
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
