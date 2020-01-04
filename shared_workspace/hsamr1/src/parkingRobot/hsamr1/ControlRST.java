package parkingRobot.hsamr1;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IControl.ControlMode;
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
public class ControlRST implements IControl {
	
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
	ControlThread ctrlThread = null;
	
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
		if (this.velocity < 0) {
			this.enteringRouteAngle = this.enteringRouteAngle + Math.PI;
		}
		this.enteringRouteAngle = (this.enteringRouteAngle + 2*Math.PI) % (2*Math.PI);
		if (this.enteringRouteAngle > 1.8*Math.PI) {
			this.enteringRouteAngle = this.enteringRouteAngle - 2*Math.PI;
		}
	}
	
	Point centerPoint;
	double trajectory_a = 0.0;
	double trajectory_c = 0.0;
	/**
	 * set parking data
	 * @see parkingRobot.IControl#setParkingData(Pose startPose, Pose endPose)
	 */
	public void setParkingData(Pose startPose, Pose endPose) {
		setDestination(startPose.getHeading(), endPose.getX(), endPose.getY());
		// calculate central point of trajectory
		this.centerPoint = endPose.getLocation().add(startPose.getLocation()).multiply((float)0.5);
		// local coordinates: translate absolute coordinates into centerPoint
		endPose.setLocation(endPose.getLocation().subtract(this.centerPoint));
		
		
		// rotate local coordinates
		if(startPose.getHeading() == 0) {
			// do not rotate
		}
		else if(Math.abs(startPose.getHeading() - Math.PI/2) < 0.001) {
			endPose.getLocation().makeRightOrth();
		}
		else if(Math.abs(startPose.getHeading() - Math.PI) < 0.001) {
			endPose.setLocation(endPose.getLocation().reverse());
		}
		
		// trajectory equation: y= a*(x**3)+c*x
		// calculate a-coefficient of trajectory
		this.trajectory_a = endPose.getY()/(-2*Math.pow(endPose.getX(),3));
		// calculate c-coefficient of trajectory
		this.trajectory_c = -this.trajectory_a*3*Math.pow(endPose.getX(),2);
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
		this.drive(this.velocity, this.angularVelocity);
	}

    /**
     * driving for destination with prior in setDriveFor-method set parameters
     * enables controlled sequences of driving forward
     */    
	private void exec_SETPOSE_ALGO(){  	
		// PD-control with angularVelocity as output, deviating angle as input
		PID omegaPIDForward = new PID(0, SAMPLETIME, 12, 0, 0.01, 0, false);
    	// signs of the initial pose data for checking if destination is reached and not driving beyond
    	double signX = Math.signum(Math.signum(this.velocity)*this.destination.getX() - this.enteringPose.getX());
    	double signY = Math.signum(Math.signum(this.velocity)*this.destination.getY() - this.enteringPose.getY());
    	double signPhi = Math.signum(this.destination.getHeading() - this.enteringPose.getHeading());
    	double signEnterAng = Math.signum(this.enteringRouteAngle);
    	
    	if (	(	(signX*(this.destination.getX() - this.currentPosition.getX()) > 0.005) ||
				(signY*(this.destination.getY() - this.currentPosition.getY()) > 0.005)    )
			&& 	(this.velocity != 0)) 
		{
		    // angle for driving straight to the destination and setting it as desired angle for control
			double routeAngle = Math.atan2(this.destination.getY()-this.currentPosition.getY(), this.destination.getX()-this.currentPosition.getX()); 
			
			// emulate forward angle for driving backwards
			if (this.velocity < 0) {
				routeAngle = routeAngle + Math.PI;
			}
			
			// make angle orientation to shortest possible rotation way
			routeAngle = (routeAngle + 2*Math.PI) % (2*Math.PI);
			if (routeAngle > 1.8*Math.PI) {
				routeAngle = routeAngle - 2*Math.PI;
			}
		    
		    omegaPIDForward.updateDesiredValue(routeAngle);
		    	
		    //TODO: Winkel von 5 grad auf 1 grad für toleranz?
		    // first Rotate towards destination 
		    if ((signEnterAng*(routeAngle - this.currentPosition.getHeading()) >  Math.toRadians(5)) && (this.angularVelocity != 0)) {
		    	// for driving backwards: emulate forward omega control
		    	drive(0, this.angularVelocity); // rotate only
		    }
		    	
		    // driving forward
		    else {
			    drive(this.velocity, omegaPIDForward.runControl(this.currentPosition.getHeading())); // drive with angle control
		    }
	    }
    	
    	//	rotate only
    	else if ( ( signPhi*(this.destination.getHeading() - this.currentPosition.getHeading()) > Math.toRadians(5) ) && ( this.angularVelocity != 0 ) )
    	{
    		drive(0,this.angularVelocity);
    	}
    	
    	// stop because destination reached
    	else {
    		this.setCtrlMode(ControlMode.INACTIVE);
    	}
    	
	}
	
    
    private double getTrajectory(double x) {
		return this.trajectory_a*Math.pow(x,3)+ this.trajectory_c*x;
	}
    
	/**
	 * parking along the generated trajectory
	 */
    double etaoldPose = 0;
    double etasumPose = 0;
    PID omegaPIDParking = new PID(0, SAMPLETIME, 12, 0, 0.01, 0, false);
    
	private void exec_PARKCTRL_ALGO(){
    	this.update_SETPOSE_Parameter();
    	//omegaPIDParking.runControl(measuredValue);
    	//omegaPIDParking.updateDesiredValue(desiredValue);
		double omega;
    	final double KP = 12;
    	final double KI = 0;
    	final double KD = .01;
    	double eta;
    	double x, y, x_next, y_next;
    	double x_local, y_local;
    	
    	x = this.currentPosition.getX();
    	y = this.currentPosition.getY();
    	
    	// local coordinates: translate absolute coordinates into centerPoint
    	x_local = x - this.centerPoint.x;
    	y_local = y - this.centerPoint.y;
    	
    	// rotate local coordinates
    	if(this.destination.getHeading() == 0 ) {
    		// do not rotate
    	}else if(Math.abs(this.destination.getHeading() - Math.PI/2) < 0.001) {
    		double n = x_local;
    		x_local = y_local;
    		y_local = -n;
    	}else if(Math.abs(this.destination.getHeading() - Math.PI) < 0.001) {
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
    	
    	y_local = getTrajectory(x_local);
    	
    	// back transformation to absolute coordinates
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
    	y_next = y_local + this.centerPoint.y;
    	x_next = x_local + this.centerPoint.x;
    	
    	
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
			drive(0,omega);
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
	    	
	    	drive(this.velocity,omega);
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
 		// PID-control with angularVelocity as output, deviation from black line center via linesensors as input
 	 	PID lineFollowPIDFast = new PID(0, SAMPLETIME, 0.02, 0.0, 0.002, 999999, false); // divided Kp/25.6 since change away from direkt powerCTR
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
 		// PID-control with angularVelocity as output, deviation from black line center via linesensors as input
 	 	PID lineFollowPIDSlow = new PID(0, SAMPLETIME, 0.01, 0.0, 0.06, 999999, true); // divided Kp/25.6 since change away from direkt powerCTR
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
	
	/* variables for drive method */
	// one PID control for each motor, outputs pwm value, input RPM
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
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * also contains underlying control of angular velocity
     * @param desiredVelocity velocity of the robot in cm/s
     * @param desiredAngularVelocity angle velocity of the robot in rad/s
     */
	private void drive(double desiredVelocity, double desiredAngularVelocity){	    
		AngleDifferenceMeasurement leftAngle = this.angleMeasurementLeft;
		AngleDifferenceMeasurement rightAngle = this.angleMeasurementRight;
		
		leftMotor.forward();
		rightMotor.forward();
		
		/* feed forward control*/
		desiredRPMLeft = (desiredVelocity-(desiredAngularVelocity*(WHEELDISTANCE/2.0)/10.0))/(WHEELDIA*Math.PI/(10.0*60.0));
		desiredRPMRight = (desiredVelocity+(desiredAngularVelocity*(WHEELDISTANCE/2.0)/10.0))/(WHEELDIA*Math.PI/(10.0*60.0)); 
		// power values from sampled linear regression
		desiredPowerLeft = (int) (0.72762 * desiredRPMLeft + 8.61696);
		desiredPowerRight = (int) (0.77850 * desiredRPMRight + 8.40402);
		
		// RPM from angle difference and conversion from deg/s to RPM 
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
