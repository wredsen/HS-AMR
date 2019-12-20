package parkingRobot.hsamr1;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
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
public class ControlRST_Ver2 implements IControl {
	
	/* geometric constants of the robot */
	final double WHEELDIA = 56; // in mm
	final double WHEELDISTANCE = 150; // in mm
	
	final double SAMPLETIME = 0.032; // in seconds
	
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
	ControlThread_Ver2 ctrlThread = null;
	
	/* class variables of desired velocities */
	double velocity = 10.0;	// in cm/s
	double angularVelocity = 1.0;	// in rad/s
	
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	/* pose Data while entering the SETPOSE-mode */ 
	Pose enteringPose = new Pose();
	double enteringRouteAngle = 0;
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
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
	public ControlRST_Ver2(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
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
		
		this.ctrlThread = new ControlThread_Ver2(this);
		
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
	 * set drive for
	 * @see parkingRobot.IControl#setDriveFor(double x, double y, double phi, double v, double w, Pose startPose)
	 */
	public void setDriveFor(double x, double y, double phi, double v, double w, Pose enteringPose) {
		setDestination(enteringPose.getHeading()+phi, enteringPose.getX() + x, enteringPose.getY() + y);
		setVelocity(v);
		setAngularVelocity(w);
		this.enteringPose = enteringPose;
		this.enteringRouteAngle = Math.atan2(this.destination.getY()-this.enteringPose.getY(), this.destination.getX()-this.enteringPose.getX());
	}
	
	Point offset;
	double trajectory_a = 0.0;
	double trajectory_c = 0.0;
	/**
	 * set parking data
	 * @see parkingRobot.IControl#setParkingData(Pose startPose, Pose endPose)
	 */
	public void setParkingData(Pose startPose, Pose endPose) {
		 // Check angle of the parking slot
		float angle = startPose.getHeading();
		// Set end position
		setDestination(angle, endPose.getX(), endPose.getY());
		//Gleichung: y= a(x+b)^3+c(x+b)+d
		// SE: start end point
//		Point deltaSE = endPose.getLocation().subtract(startPose.getLocation());
//		this.offset = endPose.getLocation().subtract(deltaSE.multiplyBy(.5f));
		// get offset
		this.offset = endPose.getLocation().add(startPose.getLocation()).multiply(.5f);
		// local coordinate
		endPose.setLocation(endPose.getLocation().subtract(this.offset));
		
		
		// Parking slot rotation
		if(angle == 0) {
			// do nothing
		}
		else if(Math.abs(angle - Math.PI/2) < 0.0001) {
			endPose.getLocation().makeRightOrth();
		}else if(Math.abs(angle - Math.PI) < 0.0001) {
			endPose.setLocation(endPose.getLocation().reverse());
		}
		
		// a variable
		this.trajectory_a = endPose.getY()/(-2*Math.pow(endPose.getX(),3));
		RConsole.println("a: " +trajectory_a );
		// c variable
		this.trajectory_c = -this.trajectory_a*3*Math.pow(endPose.getX(),2);
		RConsole.println("c: " +trajectory_c );
	}
	
	/**
	 * set pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
	
	/**
	 * get ctrl mode
	 * @see parkingRobot.IControl#getCtrlMode()
	 */
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
			case FAST		:	update_LINECTRL_Parameter();
								exec_FAST_ALGO();
								break;
			case SLOW		:	update_LINECTRL_Parameter();
								exec_SLOW_ALGO();
								break;
			case VW_CTRL	:	update_VWCTRL_Parameter();
				   				exec_VWCTRL_ALGO();
				   				break; 
			case SETPOSE	: 	update_SETPOSE_Parameter();
					  			exec_SETPOSE_ALGO();
					  			break;
			case PARK_CTRL	:	update_PARKCTRL_Parameter();
				  				exec_PARKCTRL_ALGO();
				  				break;		  					  
			case INACTIVE 	: 	exec_INACTIVE();
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
		//redundant method already done in setParkingData and exec_PARKCTRL_ALGO
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
	 * the robot can be driven with velocity in cm/s or angular velocity in rad/s during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity, 0);
	}

    /**
     * driving for destination with prior in setDriveFor-method set parameters
     * enables controlled sequences of driving forward
     */
    private void exec_SETPOSE_ALGO(){
    	// PD-control with angularVelocity as output, deviating angle as input
    	PID_Ver2 omegaPID = new PID_Ver2(0, SAMPLETIME, 12, 0, 0.01, 0, false);
    	
    	// signs of the initial pose data for checking if destination is reached and not driving beyond
    	double signX = Math.signum(this.destination.getX() - this.enteringPose.getX());
    	double signY = Math.signum(this.destination.getY() - this.enteringPose.getY());
    	double signPhi = Math.signum(this.destination.getHeading() - this.enteringPose.getHeading());
    	double signEnterAng = Math.signum(this.enteringRouteAngle);
    	
    	if (	(	signX*(this.destination.getX() - this.currentPosition.getX()) > 0.005 ||
    				signY*(this.destination.getY() - this.currentPosition.getY()) > 0.005    )
    			&& 	this.velocity != 0) 
    	{
		    // angle for driving straight to the destination and setting it as desired angle for control
		    double routeAngle = Math.atan2(this.destination.getY()-this.currentPosition.getY(), this.destination.getX()-this.currentPosition.getX()); 
		   
		    //TODO: Fixt das hier das rückwärts fahren?, eventuell noch Vorzeichen von omega bei Drive ändern falls v negativ
		    // for driving backwards: emulate forward angle
		    if (this.velocity < 0) {
		    	routeAngle = (routeAngle + Math.PI) % (2*Math.PI);
		    	signEnterAng = Math.signum((this.enteringRouteAngle + Math.PI) % (2*Math.PI));
		    }
		    omegaPID.updateDesiredValue(routeAngle);
		    	
		    // first Rotate towards destination 
		    if ((signEnterAng*(routeAngle - this.currentPosition.getHeading()) >  Math.toRadians(5)) && this.angularVelocity != 0) {
		    	// for driving backwards: emulate forward omega control
		    	drive(0, this.angularVelocity, 0); // rotate only
		    }
		    	
		    // driving forward
		    else {
			    drive(this.velocity, omegaPID.runControl(this.currentPosition.getHeading()), 0); // drive with angle control
		    }
	    }
    	
    	//	rotate only
    	else if (signPhi*(this.destination.getHeading() - this.currentPosition.getHeading()) > Math.toRadians(5) && this.angularVelocity != 0)
    	{
    		drive(0,this.angularVelocity, 0);
    	}
    	
    	// stop because destination reached
    	else {
    		this.setCtrlMode(ControlMode.INACTIVE);
    	}
    	
	}
	
    
    private double calculateLocalPolynome(double x) {
		return this.trajectory_a*Math.pow(x,3)+ this.trajectory_c*x;
	}
    double etaoldPose = 0;
    double etasumPose = 0;
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//PID_Ver2 omegaPID = new PID_Ver2(0, SAMPLETIME, 12, 0, 0.01, 0, false);

    	this.update_SETPOSE_Parameter();
    	
		double omega;
    	final double KP = 12;
    	final double KI = 0;
    	final double KD = .01;
    	double eta;
    	double x, y, x_next, y_next;
    	double x_local, y_local;
    	
    	x = this.currentPosition.getX();
    	y = this.currentPosition.getY();
    	
    	// make local
    	x_local = x - this.offset.x;
    	y_local = y - this.offset.y;
    	
    	// transform coordinates
    	if(this.destination.getHeading() == 0 ) {
    		// do nothing
    	}else if(Math.abs(this.destination.getHeading() - Math.PI/2) < 0.0001) {
    		double n = x_local;
    		x_local = y_local;
    		y_local = -n;
    	}else if(Math.abs(this.destination.getHeading() - Math.PI) < 0.0001) {
    		x_local = -x_local;
    		y_local = -y_local;
    	}
    	
    	//TODO: Das hier rausnehmen oder inkrement erhöhen -> von x abhängig machen?
    	// Check for driving direction
    	if(Math.signum(this.velocity) == 1){
    		x_local = x_local + 0.01;
    	}else if(Math.signum(this.velocity) == -1) {
    		x_local = x_local - 0.01;
    	}
    	
    	y_local = calculateLocalPolynome(x_local);
    	
    	// back transformation
    	if(this.destination.getHeading() == 0 ) {
    		// do nothing
    	}else if(Math.abs(this.destination.getHeading() - Math.PI/2) < 0.0001) {
    		double n = x_local;
    		x_local = -y_local;
    		y_local = n;
    	}else if(Math.abs(this.destination.getHeading() - Math.PI) < 0.0001) {
    		x_local = -x_local;
    		y_local = -y_local;
    	}
    	
    	//make global
    	y_next = y_local + this.offset.y;
    	x_next = x_local + this.offset.x;
    	
    	if(Math.abs(this.destination.getHeading() - Math.PI) < 0.0001) {
    		eta = Math.atan((y - y_next)/(x - x_next))- this.currentPosition.getHeading() + Math.PI;
    	}else if(Math.abs(this.destination.getHeading() - Math.PI/2) < 0.0001) {
    		if(Math.signum(this.velocity) == 1){
    			eta = Math.atan((y + y_next)/(x - x_next))- this.currentPosition.getHeading() + Math.PI/2;
        	}else  {
        		eta = - ( Math.atan((y + y_next)/(x - x_next))- this.currentPosition.getHeading() + Math.PI/2);
        	}
    		
    	}else {
    		eta = Math.atan((y - y_next)/(x - x_next))- this.currentPosition.getHeading();
    	}
    	//	 Check if destination is reached
    	if(this.currentPosition.getLocation().subtract(this.destination.getLocation()).length()<0.05) 
    	{
			eta = this.destination.getHeading() -this.currentPosition.getHeading();
	    	omega = KP*eta ;
	    	etaoldPose = eta;
			drive(0,omega, 0);
			//TODO: Winkel verkleinern von 0.07 abwärts
			if( Math.abs(eta) < 0.07 ){
				this.setCtrlMode(ControlMode.INACTIVE);
			}
			
    	}
    	else {
	    	etasumPose += eta;
	    	omega = KP*eta + KI*etasumPose + KD*(etaoldPose-eta);
	    	etaoldPose = eta;
	    	RConsole.println("[control] Fehler: " + omega);
	    	
	    	//TODO: DIRTY FIX funktioniert?, sonst versuchen rauszunhemen und PD werte besser zu kalibrieren
	    	if (Math.abs(this.destination.getHeading()-this.navigation.getPose().getHeading()) > Math.toRadians(50)) {
	    		omega = -omega;
	    	}if (Math.abs(omega) > Math.toRadians(40)) {
	            omega = Math.signum(omega)*Math.toRadians(40);
	        }
	    	
	    	drive(this.velocity,omega, 0);
	    }

	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * with high speed of 15 cm/s and low d-control
	 */
    private void exec_FAST_ALGO() {
    	leftMotor.forward();
		rightMotor.forward();
		double desiredVelocity = 15;
		// PID-control with angularVelocity as output, deviation from black line center via linesensors as input
		PID_Ver2 lineFollowPIDFast = new PID_Ver2(0, SAMPLETIME, 0.2, 0, 0.05, 999999, false);
		int lineControlFast = (int) lineFollowPIDFast.runControl(this.lineSensorLeft - this.lineSensorRight);
		drive(desiredVelocity, 0, lineControlFast);	
    }
    
    /**
	 * DRIVING along black line
	 * with low speed of 7 cm/s and high d-control
	 */
    private void exec_SLOW_ALGO() {
    	leftMotor.forward();
		rightMotor.forward();
		double desiredVelocity = 7;
		// PID-control with angularVelocity as output, deviation from black line center via linesensors as input
		PID_Ver2 lineFollowPIDSlow = new PID_Ver2(0, SAMPLETIME, 0.3, 0.0, 0.13, 999999, true);
		int lineControlSlow = (int) lineFollowPIDSlow.runControl(this.lineSensorLeft - this.lineSensorRight);
		drive(desiredVelocity, 0, lineControlSlow);
    }
    
	/** 
	 * stopping all kinds of the robot's movement immediately
	 */
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * also contains underlying control of angular velocity
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */
	private void drive(double v, double w, int control){
		double desiredVelocity = v; // in cm/s
		double desiredAngularVelocity = w; // in 1/s
		
	    PID_Ver2 leftRPMPID = new PID_Ver2(0, SAMPLETIME, 0.6, 0.2, 0.005, 99999, false);
	    PID_Ver2 rightRPMPID = new PID_Ver2(0, SAMPLETIME, 0.6, 0.2, 0.005, 99999, false); 
	    
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
		
		/* feed forward control*/
		desiredRPMLeft = (desiredVelocity-(desiredAngularVelocity*(WHEELDISTANCE/2)/10))/(WHEELDIA*Math.PI/(10.0*60.0));
		desiredRPMRight = (desiredVelocity+(desiredAngularVelocity*(WHEELDISTANCE/2)/10))/(WHEELDIA*Math.PI/(10.0*60.0)); 
		// power values from sampled linear regression
		desiredPowerLeft = (int) (0.72762 * desiredRPMLeft + 8.61696);
		desiredPowerRight = (int) (0.77850 * desiredRPMRight + 8.40402);
		
		// RPM from angle difference and conversion from deg/s to RPM 
		measuredRPMLeft = ((double) leftAngle.getAngleSum() / (double) leftAngle.getDeltaT()) * 166.667; //in revelations per min
		measuredRPMRight = ((double) rightAngle.getAngleSum() / (double) rightAngle.getDeltaT()) * 166.667; //in revelations per min
		
		leftRPMPID.updateDesiredValue(desiredRPMLeft - control);
		rightRPMPID.updateDesiredValue(desiredRPMRight + control);
		
		leftControlOut = (int) leftRPMPID.runControl(measuredRPMLeft);
		rightControlOut = (int) rightRPMPID.runControl(measuredRPMRight);
		
		leftMotor.setPower(desiredPowerLeft + leftControlOut);
		rightMotor.setPower(desiredPowerRight + rightControlOut);
	}
}

/*	geändert:
 * 
 * TODO:	Regelausgang bei drive(..,omega,control) in omega packen 
 */
