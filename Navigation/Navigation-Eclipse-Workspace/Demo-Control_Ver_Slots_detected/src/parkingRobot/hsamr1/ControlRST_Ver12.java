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
public class ControlRST_Ver12 implements IControl {
	
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
	ControlThread_Ver12 ctrlThread = null;
	
	double velocity = 10.0;	// in cm/s
	double angularVelocity = 0.10;	// in 1/s
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	Pose enteringPosition = new Pose();
	double enteringEta = 0;
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    
	final double SAMPLETIME = 0.032; // in seconds
	final double WHEELDIA = 56; // in mm
	final double WHEELDISTANCE = 150; // in mm
	
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
	public ControlRST_Ver12(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
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
		
		this.ctrlThread = new ControlThread_Ver12(this);
		
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
	
	public void setDriveFor(double x, double y, double phi, double v, double omega, Pose enteringPosition) {
		setDestination(enteringPosition.getHeading()+phi, enteringPosition.getX() + x, enteringPosition.getY() + y);
		setVelocity(v);
		setAngularVelocity(omega);
		this.enteringPosition = enteringPosition;
		this.enteringEta = Math.atan2(this.destination.getY()-this.enteringPosition.getY(), this.destination.getX()-this.enteringPosition.getX());
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
	
	public ControlMode getCtrlMode() {
		return this.currentCTRLMODE;
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
			case FAST	:	update_LINECTRL_Parameter();
							exec_FAST_ALGO();
							break;
			case SLOW	:	update_LINECTRL_Parameter();
							exec_SLOW_ALGO();
							break;
			case VW_CTRL	:	update_VWCTRL_Parameter();
				   				exec_VWCTRL_ALGO();
				   				break; 
			case SETPOSE	: 	update_SETPOSE_Parameter();
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
		this.drive(this.velocity, this.angularVelocity, 0);
	}

    private void exec_SETPOSE_ALGO(){

    	PID_Ver12 omegaPID = new PID_Ver12(0, SAMPLETIME, 7, 0, 0.01, 0, false); // 0.3, 0, 0.1, 2 // 0.7
    	double signX = Math.signum(this.destination.getX() - this.enteringPosition.getX());
    	double signY = Math.signum(this.destination.getY() - this.enteringPosition.getY());
    	double signPhi = Math.signum(this.destination.getHeading() - this.enteringPosition.getHeading());
    	double signEta = Math.signum(this.enteringEta);
    	
    	this.update_SETPOSE_Parameter();
    	//LCD.clear();
		//LCD.drawString("akt phi:"+ currentPosition.getHeading(), 0, 3);
		//LCD.drawString("Ziel phi:"+ destination.getHeading() + " " + destination.getX() + " " + destination.getY(), 0, 4);
		//LCD.drawString("akt:" + currentPosition.getX() + " " + currentPosition.getY(), 0, 6);
		//LCD.drawString("Ziel:"+ destination.getX() + " " + destination.getY(), 0, 7);
    	
    	double omega = this.angularVelocity;
    	double eta;
    	
    	if ((	signX*(this.destination.getX() - this.currentPosition.getX()) > 0.005 ||
    			signY*(this.destination.getY() - this.currentPosition.getY()) > 0.005    )
    			&& this.velocity != 0) 
    	{
	    	// Angle for driving to destination point
	    	double angleCourse = Math.atan2(this.destination.getY()-this.currentPosition.getY(), this.destination.getX()-this.currentPosition.getX());
	    	// Diff-Angle
	    	eta = angleCourse - this.currentPosition.getHeading();
	    	omegaPID.updateDesiredValue(eta);
	    	
	    	
	    	// Translate & Rotate
	    	if ((signEta*eta >  Math.toRadians(2)) && this.angularVelocity != 0) // only turn
	    	{
	    		drive(0,this.angularVelocity, 0);
	    	}
	    	
	    	// Translate only
	    	else
	    	{
	    		// drive with angle control
		    	omega = omegaPID.runControl(this.currentPosition.getHeading());
		    	drive(this.velocity,omega, 0);
		    	LCD.clear(6);
		    	LCD.drawString("CTR_Error:"+ omega, 0, 6);
	    	}
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
    private void exec_FAST_ALGO() {

    	leftMotor.forward();
		rightMotor.forward();
		PID_Ver12 lineFollowPIDFast = new PID_Ver12(0, SAMPLETIME, 0.2, 0, 0.05, 999999, false);
		double desiredVelocity = 20;
		int lineControlFast = (int) lineFollowPIDFast.runControl(this.lineSensorLeft - this.lineSensorRight);
		drive(desiredVelocity, 0, lineControlFast);
		
		//LCD.clear();
		//LCD.drawString("X:" + navigation.getPose().getX(), 0, 2);
		//LCD.drawString("Y:" + navigation.getPose().getY(), 0, 3);
		//LCD.drawString("C Num = " + this.navigation.getCornerNumber(), 0,5);		
    }
    
    private void exec_SLOW_ALGO() {
    	
    	leftMotor.forward();
		rightMotor.forward();
		PID_Ver12 lineFollowPIDSlow = new PID_Ver12(0, SAMPLETIME, 0.3, 0.0, 0.13, 999999, true);	// D: 0.09
		double desiredVelocity = 7;
		int lineControlSlow = (int) lineFollowPIDSlow.runControl(this.lineSensorLeft - this.lineSensorRight);
		drive(desiredVelocity, 0, lineControlSlow);
		//LCD.clear();
		//LCD.drawString("X:" + navigation.getPose().getX(), 0, 2);
		//LCD.drawString("Y:" + navigation.getPose().getY(), 0, 3);
		//LCD.drawString("C Num = " + this.navigation.getCornerNumber(), 0,5);		
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
	private void drive(double v, double omega, int control){
		//Aufgabe 3.2
		double desiredVelocity = v; // in cm/s
		double desiredAngularVelocity = omega; // in 1/s
		
	    PID_Ver12 leftRPMPID = new PID_Ver12(0, SAMPLETIME, 0.6, 0.2, 0.005, 99999, false); //P:0.5, I:0.3
	    PID_Ver12 rightRPMPID = new PID_Ver12(0, SAMPLETIME, 0.6, 0.2, 0.005, 99999, false); //P:0.5, I:0.3
	    
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
		desiredRPMLeft = (desiredVelocity-(desiredAngularVelocity*(WHEELDISTANCE/2)/10))/(WHEELDIA*Math.PI/(10.0*60.0));
		desiredRPMRight = (desiredVelocity+(desiredAngularVelocity*(WHEELDISTANCE/2)/10))/(WHEELDIA*Math.PI/(10.0*60.0)); 
		desiredPowerLeft = (int) (0.71270 * desiredRPMLeft + 9.0397);
		desiredPowerRight = (int) (0.81097 * desiredRPMRight + 9.12246);
		
			
		
		measuredRPMLeft = ((double) leftAngle.getAngleSum() / (double) leftAngle.getDeltaT()) * 166.667; //in revelations per min
		measuredRPMRight = ((double) rightAngle.getAngleSum() / (double) rightAngle.getDeltaT()) * 166.667; //in revelations per min
		
		leftRPMPID.updateDesiredValue(desiredRPMLeft - control);
		rightRPMPID.updateDesiredValue(desiredRPMRight + control);
		
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

/*	geändert: ThreadSleeps angepasst, PID ADRW geändert, vorher e1-e2
 * 
 * TODO: 
 * 			Transition von Driving in TURNING kontrollieren
 * 			Regelausgang bei drive(..,omega,control) in omega packen 
 */
