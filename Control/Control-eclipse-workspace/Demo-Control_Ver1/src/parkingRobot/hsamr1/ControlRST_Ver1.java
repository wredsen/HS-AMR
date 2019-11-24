package parkingRobot.hsamr1;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IControl.ControlMode;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;
import lejos.nxt.Sound;
import lejos.nxt.comm.RConsole;
import lejos.geom.Point;

/**
 * Main class for control module
 *
 */
public class ControlRST_Ver1 implements IControl {
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an l	ast request
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
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread_Ver1 ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 10.0;	// in cm/s
	double angularVelocity = 0.10;	// in 1/s
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    
	double rpmSampleTime = 0.054; // in seconds
	final double wheelDiameter = 56; // in mm
	final double wheelDistance = 150; // in mm
	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST_Ver1(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		this.lineSensorRight		= perception.getRightLineSensor();
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		
		this.ctrlThread = new ControlThread_Ver1(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY-1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
		
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		this.angleMeasurementLeft = encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight = encoderRight.getEncoderMeasurement();
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensorValue();
		this.lineSensorLeft  		= perception.getLeftLineSensorValue();		
		this.currentPosition		= navigation.getPose();
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity);
	}
    
	// UNBEDINGT REWORKEN: gehackt fuer 1. Verteidigung
    double drivingOffset = 0.1;
	double drivingDistance = 1.2+drivingOffset; // Angabe in Metern (120 cm forward @ 10 cm/s)
    double rotatingOffset = 0.45;
    double rotatingDistance = -(Math.PI/2 + rotatingOffset);	//(90 deg @ 30 deg/s)
    private void exec_SETPOSE_ALGO(){
    	
    	PID_Ver1 omegaPID = new PID_Ver1(0, rpmSampleTime, 0.1, 0, 0.05, 2);
    	this.setAngularVelocity(1.0);
    	
    	this.update_SETPOSE_Parameter();
    	this.setDestination(0, drivingDistance, 0);
    	LCD.clear();
		LCD.drawString("akt phi:"+ currentPosition.getHeading(), 0, 3);
		LCD.drawString("Ziel phi:"+ destination.getHeading() + " " + destination.getX() + " " + destination.getY(), 0, 4);
		LCD.drawString("akt:" + currentPosition.getX() + " " + currentPosition.getY(), 0, 6);
		LCD.drawString("Ziel:"+ destination.getX() + " " + destination.getY(), 0, 7);
    	
    	this.setVelocity(10);
    	double omega = this.angularVelocity;
    	double eta;
    	
    	if ((Math.abs(this.destination.getX() - this.currentPosition.getX()) > 0.1 ||
    			Math.abs(this.destination.getY() - this.currentPosition.getY()) > 0.1)
    			&& this.velocity != 0) 
    	{
	    	// Angle for driving to destination point
	    	double angleCourse = Math.atan2(this.destination.getY()-this.currentPosition.getY(), this.destination.getX()-this.currentPosition.getX());
	    	eta = angleCourse - this.currentPosition.getHeading();
	    	omegaPID.updateDesiredValue(eta);
	    	
	    	if (eta >  Math.toRadians(5) && this.angularVelocity != 0) // only turn
	    	{
	    		drive(0,this.angularVelocity);
	    	}
	  
	    	else if (this.velocity != 0)// drive with angle regulation
	    	{
		    	omega = omegaPID.runControl(this.currentPosition.getHeading());
		 
		    	RConsole.println("[control] Fehler: " + omega);
		    	drive(this.velocity,omega);
	    	}
    	}
    	else if (Math.abs(this.destination.getHeading() - this.currentPosition.getHeading()) > Math.toRadians(5) && this.angularVelocity != 0)
    	{
    		drive(0,this.angularVelocity);
	    	// Destination angle
	    	eta = this.destination.getHeading() - this.currentPosition.getHeading();
    	}
    	else {
    		this.setCtrlMode(ControlMode.INACTIVE);
    	}
    	
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
	private void exec_LINECTRL_ALGO(){	    
		
		PID_Ver1 lineFollowPIDSlow = new PID_Ver1(0, rpmSampleTime, 0.1, 0, 0.01, 999999);
		PID_Ver1 lineFollowPIDFast = new PID_Ver1(0, rpmSampleTime, 0.2, 0, 0.005, 999999);
		int BASEPOWER = 0;
		leftMotor.forward();
		rightMotor.forward();
		
		/*
		Regelgröße: Lichtintensität linker und rechter Sensor
		Regelfehler: Sollwert (0) - Istwert der Differenz beider Sensorwerte (lineSensorLeft - lineSensorRight)
		*/
		int lineControlSlow = (int) lineFollowPIDSlow.runControl(this.lineSensorLeft - this.lineSensorRight);
		int lineControlFast = (int) lineFollowPIDFast.runControl(this.lineSensorLeft - this.lineSensorRight);
		// das hier später an drehzahlregelung einzelner räder übergeben
		
		double desiredVelocity = velocity; 
		
		double desiredRPMLeft = desiredVelocity/(wheelDiameter*Math.PI/(10.0*60.0));
		double desiredRPMRight = desiredVelocity/(wheelDiameter*Math.PI/(10.0*60.0)); 
		int desiredPowerLeft = (int) (0.66242 * desiredRPMLeft + 11.86405);
		int desiredPowerRight = (int) (0.70069 * desiredRPMRight + 15.155);
		
		if (this.navigation.getCornerArea() == true) {
			BASEPOWER = 30;
			LCD.clear();
			LCD.drawString("CornerArea", 0, 5);
			if (this.navigation.getCorner() == true) {
				drive(0, 0.07);
				Sound.twoBeeps();
			}
			
			else {
				rightMotor.setPower(BASEPOWER + lineControlSlow);
				leftMotor.setPower(BASEPOWER - lineControlSlow);
			}		
		}
		
		else {
			BASEPOWER = 50;
			rightMotor.setPower(BASEPOWER + lineControlFast);
			leftMotor.setPower(BASEPOWER - lineControlFast);
		}
	}
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */
	private void drive(double v, double omega){
		//Aufgabe 3.2
		double desiredVelocity = v; // in cm/s
		double desiredAngularVelocity = omega; // in 1/s
		
	    PID_Ver1 leftRPMPID = new PID_Ver1(0, rpmSampleTime, 0.6, 0.2, 0.005, 99999); //P:0.5, I:0.3
	    PID_Ver1 rightRPMPID = new PID_Ver1(0, rpmSampleTime, 0.7, 0.3, 0.005, 99999); //P:0.5, I:0.3
	    
	    int leftControlOut = 0;
	    int rightControlOut = 0;
	    double measuredRPMLeft = 0;
	    double measuredRPMRight = 0;
		
		double desiredRPMLeft = 0;
		double desiredRPMRight = 0;
		int desiredPowerLeft = 0;
		int desiredPowerRight = 0;
		
		AngleDifferenceMeasurement leftAngle = this.angleMeasurementLeft;
		AngleDifferenceMeasurement rightAngle = this.angleMeasurementRight;
		
		leftMotor.forward();
		rightMotor.forward();
		
		
		// MONITOR (example)
		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
		
		/* Vorsteuerung*/
		desiredRPMLeft = (0.5*desiredVelocity-(desiredAngularVelocity*(wheelDistance/2)/(2*10)))/((wheelDiameter/2)*Math.PI/(10.0*60.0));
		desiredRPMRight = (0.5*desiredVelocity+(desiredAngularVelocity*(wheelDistance/2)/(2*10)))/((wheelDiameter/2)*Math.PI/(10.0*60.0)); 
		desiredPowerLeft = (int) (0.66242 * desiredRPMLeft + 11.86405);
		desiredPowerRight = (int) (0.70069 * desiredRPMRight + 15.155);
			
		
		measuredRPMLeft = ((double) leftAngle.getAngleSum() / (double) leftAngle.getDeltaT()) * 166.667; //in revelations per min
		measuredRPMRight = ((double) rightAngle.getAngleSum() / (double) rightAngle.getDeltaT()) * 166.667; //in revelations per min
		
		leftRPMPID.updateDesiredValue(desiredRPMLeft);
		rightRPMPID.updateDesiredValue(desiredRPMRight);
		
		leftControlOut = (int) leftRPMPID.runControl(measuredRPMLeft);
		rightControlOut = (int) rightRPMPID.runControl(measuredRPMRight);
		
		leftMotor.setPower(desiredPowerLeft + leftControlOut);
		rightMotor.setPower(desiredPowerRight + rightControlOut);
		
		/*
		LCD.clear();
		LCD.drawString("DesRPMLeft:"+desiredRPMLeft, 0, 1);
		LCD.drawString("DesRPMRight:"+desiredRPMRight, 0, 2);
		LCD.drawString("MeasRPMLeft:"+measuredRPMLeft, 0, 3);
		LCD.drawString("MeasRPMRight:"+measuredRPMRight, 0, 4);
		LCD.drawString("RegLeft:"+leftControlOut, 0, 5);
		LCD.drawString("RegRight:"+rightControlOut, 0, 6);
		*/
		//LCD.drawString("SampleTime:"+this.encoderRight.getEncoderMeasurement().getDeltaT(), 0, 7);
	}
}

