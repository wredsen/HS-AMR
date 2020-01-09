package parkingRobot.hsamr1;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;
import lejos.geom.Point;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	/* geometric constants of the robot */
	final double WHEELDIA = 56; // in mm
	final double WHEELDISTANCE = 150; // in mm
	
	/* time between each linesensor sample */
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
	ControlThread ctrlThread = null;	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	/* storing start time */
	int lastTime = 0;
	
	/* class variables for desired velocities */
	double velocity = 10.0;	// in cm/s
	double angularVelocity = 1.0;	// in rad/s
	
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	/* current pose while entering the SETPOSE-mode */ 
	Pose enteringPose = new Pose();
	double enteringRouteAngle = 0;
	
	/* center point and coefficients of calculated trajectory for parking-mode */
	Point centerPoint;
	double trajectoryCoeffA = 0.0;
	double trajectoryCoeffC = 0.0;
	
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
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
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
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	/************************* public input methods *****************************/ 
	
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
	 * sets destination and speed for straight trajectory of SETPOSE-mode
	 * @param x relative x destination in m
	 * @param y relative y destination in m
	 * @param phi relative heading in rad
	 * @param v translatory velocity in cm/s
	 * @param w angular velocity in rad/s
	 * @param enteringPose current position while starting SETPOSE-mode
	 */
	public void setDriveFor(double x, double y, double phi, double v, double w, Pose enteringPose) {
		setDestination(enteringPose.getHeading()+phi, enteringPose.getX() + x, enteringPose.getY() + y);
		setVelocity(v);
		setAngularVelocity(w);
		this.enteringPose = enteringPose;
		this.enteringRouteAngle = Math.atan2(this.destination.getY()-this.enteringPose.getY(), this.destination.getX()-this.enteringPose.getX());
		if (this.velocity < 0) {
			this.enteringRouteAngle = this.enteringRouteAngle + Math.PI;
		}
		this.enteringRouteAngle = (this.enteringRouteAngle + 2*Math.PI) % (2*Math.PI);
		if (this.enteringRouteAngle > 1.8*Math.PI) {
			this.enteringRouteAngle = this.enteringRouteAngle - 2*Math.PI;
		}
	}
	
	/**
	 * sets destination and start for polynomial trajectory of parking-mode
	 * @param startPose start pose of the polynom
	 * @param finalPose final pose of the polynom
	 */
	public void setParkingFor(Pose startPose, Pose finalPose) {
		setDestination(finalPose.getHeading(), finalPose.getX(), finalPose.getY());
		
		/*calculate central point of trajectory */
		centerPoint.x = 0.5f * (startPose.getX() + finalPose.getX());
		centerPoint.y = 0.5f * (startPose.getY() + finalPose.getY());
		
		/* local coordinates: translate absolute coordinates into centerPoint */
		finalPose.setLocation(finalPose.getLocation().subtract(centerPoint));
		
		
		/* local coordinate: rotate coordinates */
		if(Math.abs(startPose.getHeading() - Math.toRadians(90)) < 0.01) {
			finalPose.getLocation().makeRightOrth();	// top parking spot
		}
		if(Math.abs(startPose.getHeading() - Math.toRadians(180)) < 0.01) {
			finalPose.setLocation(finalPose.getLocation().reverse());	// right parking spot
		}
		
		// trajectory equation: y= a*(x**3)+c*x (like documentation)
		// calculate a-coefficient of trajectory
		trajectoryCoeffA = (-1)*finalPose.getY()/(2*Math.pow(finalPose.getX(), 3));
		// calculate c-coefficient of trajectory
		trajectoryCoeffC = -this.trajectoryCoeffA*3*Math.pow(finalPose.getX(), 2);
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
	
	/**************** Private methods ************************/
	
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
		//redundant method already done in setParkingFor and exec_PARKCTRL_ALGO
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
	 * calculating the value of the polynom y(x) = a*(x**3) + c*x 
	 */
    private double getTrajectory(double x) {
		return ( trajectoryCoeffA*Math.pow(x, 3) + trajectoryCoeffC*x );
	}
	
	/**
	 * the robot can be driven with velocity in cm/s or angular velocity in rad/s during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity);
	}

    /**
     * driving for destination with prior in setDriveFor-method set parameters
     * enables controlled sequences of driving forward
     */    
	private void exec_SETPOSE_ALGO(){  	
		this.update_SETPOSE_Parameter();
		
		/* PD-control with angularVelocity as output, deviating angle as input */
		PID omegaPIDForward = new PID(0, SAMPLETIME, 8, 0, 0.01, 0, false);
    	
		/* signs of the initial pose data for checking if destination is reached and not driving beyond */
    	double signX = Math.signum(this.destination.getX() - this.enteringPose.getX());
    	double signY = Math.signum(this.destination.getY() - this.enteringPose.getY());
    	double signPhi = Math.signum(this.destination.getHeading() - this.enteringPose.getHeading());
    	double signEnterAng = Math.signum(this.enteringRouteAngle);
    	
    	if (	(	(signX*(this.destination.getX() - this.currentPosition.getX()) > 0.005) ||
					(signY*(this.destination.getY() - this.currentPosition.getY()) > 0.005)    )
    			&& 	(this.velocity != 0)) {
		    /* angle for driving straight to the destination and setting it as desired angle for control */
			double routeAngle = Math.atan2(this.destination.getY() - this.currentPosition.getY(), this.destination.getX() - this.currentPosition.getX()); 
			
			/* emulate forward angle for driving backwards */
			if (this.velocity < 0) {
				routeAngle = routeAngle + Math.PI;
			}
			
			/* make a continuous angle orientation for rotation */
			routeAngle = (routeAngle + 2*Math.PI) % (2*Math.PI);
			if (routeAngle > 1.8*Math.PI) {
				routeAngle = routeAngle - 2*Math.PI;
			}
		    
			/* setting calculated reference value of the controller */
		    omegaPIDForward.updateDesiredValue(routeAngle);
		    	
		    /* open loop controlled rotating */
		    if ((signEnterAng*(routeAngle - this.currentPosition.getHeading()) >  Math.toRadians(5)) && (this.angularVelocity != 0)) {
		    	drive(0, this.angularVelocity);
		    }
		    	
		    /* driving forward with omega control */
		    else {
			    drive(this.velocity, omegaPIDForward.runControl(this.currentPosition.getHeading()));
		    }
	    }
    	
    	/* open loop controlled rotating only */
    	else if (( signPhi*(this.destination.getHeading() - this.currentPosition.getHeading()) > Math.toRadians(5) ) && ( this.angularVelocity != 0 )){
    		drive(0,this.angularVelocity);
    	}
    	
    	/* stop if destination reached */
    	else {
    		this.setCtrlMode(ControlMode.INACTIVE);
    	}
    	
	}
    
    /* variable for storing of last controller output */
    double parkingOmega;
	/**
	 * parking along the generated trajectory
	 */
	private void exec_PARKCTRL_ALGO(){
    	this.update_SETPOSE_Parameter();
    	
    	/* PD-control with angularVelocity as output, deviating angle as input */
    	PID omegaPIDParking = new PID(0, SAMPLETIME, 2.0, 0, 0.02, 0, false);
    	
    	/* transform into local coordinates by translating absolute coordinates into centerPoint
    	 * so variables for x- and y-coordinates of the next track section-destination can be easily calculated */
    	double nextX = this.currentPosition.getX() - this.centerPoint.x;
    	double nextY = this.currentPosition.getY() - this.centerPoint.y;	
    	
    	/* rotate local coordinates */
    	if (Math.abs(this.destination.getHeading() - Math.toRadians(90)) < 0.01){
    		double storing = nextX;
    		nextX = nextY;
    		nextY = -storing;
    	}
    	if (Math.abs(this.destination.getHeading() - Math.toRadians(180)) < 0.01){
    		nextX = -nextX;
    		nextY = -nextY;
    	}
    	
    	/* iterating over x and incrementing it */
    	nextX = nextX + 0.01;
    	/* calculating y value for this */
    	nextY = getTrajectory(nextX);
    	
    	/* transforming back to absolute coordinates by first rotating */
    	if (Math.abs(this.destination.getHeading() - Math.toRadians(90)) < 0.01){
    		double store = nextX;
    		nextX = -nextY;
    		nextY = store;
    	}
    	if (Math.abs(this.destination.getHeading() - Math.toRadians(180)) < 0.01){
    		nextX = -nextX;
    		nextY = -nextY;
    	}
    	
    	/* transforming back to absolute coordinates by translating */
    	nextY = nextY + this.centerPoint.y;
    	nextX = nextX + this.centerPoint.x;
    	
    	/* calculating reference variable */
    	double routeAngle = 0;
    	
    	//check parking orientation
    	if (Math.abs(this.destination.getHeading() - Math.toRadians(180)) < 0.01){
    		routeAngle = Math.atan((this.currentPosition.getY() - nextY)/(this.currentPosition.getX() - nextX)) + Math.PI;
    	}
    	else {
    		routeAngle = Math.atan2(-(this.currentPosition.getY() - nextY), -(this.currentPosition.getX() - nextX));
    	}
    	
    	omegaPIDParking.updateDesiredValue(routeAngle);
    	
    	//	 Check if destination is reached
    	if (this.currentPosition.getLocation().subtract(this.destination.getLocation()).length()<0.05) {
    		
    		routeAngle = this.destination.getHeading();
    		drive(0,Math.signum(parkingOmega) * Math.toRadians(40));
    		
			if (Math.abs(destination.getHeading() - this.currentPosition.getHeading()) < Math.toRadians(2)) {
				this.setCtrlMode(ControlMode.INACTIVE);				
			}
    	}
    	else {
	    	parkingOmega = omegaPIDParking.runControl(this.currentPosition.getHeading());
	    	drive(this.velocity, parkingOmega);
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
 		/* PID-control with angularVelocity as output, deviation from black line center via linesensors as input */
 	 	PID lineFollowPIDFast = new PID(0, SAMPLETIME, 0.02, 0.0, 0.002, 999999, false);
    	leftMotor.forward();
		rightMotor.forward();
		double desiredVelocity = 15;
		double omegaPIDFast = lineFollowPIDFast.runControl(this.lineSensorLeft - this.lineSensorRight);
		drive(desiredVelocity, omegaPIDFast);	
    }
    
    /**
	 * DRIVING along black line
	 * with low speed of 7 cm/s and high d-control
	 */ 
 	private void exec_SLOW_ALGO() {
 		/* PID-control with angularVelocity as output, deviation from black line center via linesensors as input */
 	 	PID lineFollowPIDSlow = new PID(0, SAMPLETIME, 0.01, 0.0, 0.06, 999999, true);
 		leftMotor.forward();
		rightMotor.forward();
		double desiredVelocity = 7;
		double omegaPIDSlow = lineFollowPIDSlow.runControl(this.lineSensorLeft - this.lineSensorRight);
		drive(desiredVelocity, omegaPIDSlow);
    }
    
	/** 
	 * stopping all kind of the robot's movement immediately
	 */
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
	

    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * also contains underlying control of angular velocity
     * @param desiredVelocity velocity of the robot in cm/s
     * @param desiredAngularVelocity angle velocity of the robot in rad/s
     */
	private void drive(double desiredVelocity, double desiredAngularVelocity){	
		/* variables for drive method */
		/* one PID control for each motor, outputs pwm value, input RPM */
	    PID leftPIDRPM = new PID(0, SAMPLETIME, 0.6, 0.2, 0.005, 99999, false);
	    PID rightPIDRPM = new PID(0, SAMPLETIME, 0.6, 0.2, 0.005, 99999, false); 
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
		desiredRPMLeft = (desiredVelocity-(desiredAngularVelocity*(WHEELDISTANCE/2.0)/10.0))/(WHEELDIA*Math.PI/(10.0*60.0));
		desiredRPMRight = (desiredVelocity+(desiredAngularVelocity*(WHEELDISTANCE/2.0)/10.0))/(WHEELDIA*Math.PI/(10.0*60.0));
		
		/* power values from sampled linear regression */
		desiredPowerLeft = (int) (0.72762 * desiredRPMLeft + 8.61696);
		desiredPowerRight = (int) (0.77850 * desiredRPMRight + 8.40402);
		
		/* RPM from angle difference and conversion from deg/s to RPM */
		measuredRPMLeft = ((double) leftAngle.getAngleSum() / (double) leftAngle.getDeltaT()) * 166.667; //in revelations per min
		measuredRPMRight = ((double) rightAngle.getAngleSum() / (double) rightAngle.getDeltaT()) * 166.667; //in revelations per min
		
		leftPIDRPM.updateDesiredValue(desiredRPMLeft);
		rightPIDRPM.updateDesiredValue(desiredRPMRight);
		
		leftControlOut = (int) leftPIDRPM.runControl(measuredRPMLeft);
		rightControlOut = (int) rightPIDRPM.runControl(measuredRPMRight);
		
		leftMotor.setPower(desiredPowerLeft + leftControlOut);
		rightMotor.setPower(desiredPowerRight + rightControlOut);
	}
}
