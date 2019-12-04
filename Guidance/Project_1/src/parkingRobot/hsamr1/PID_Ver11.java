package parkingRobot.hsamr1;

public class PID_Ver11 {
	
	double desiredValue = 0;
	double samplePeriod = 0;
	
	double Kr = 0;
	double Ki = 0;
	double Kd = 0;
	double AIWR = 99999999;	//Anti-I-Reset-Windup
	boolean ADWR = false;	//Anti-D-Reset-Windup	
	
	double errorValue0 = 0; 
	double errorValue1 = 0;	//e[k-1]
	double errorValue2 = 0; //e[k-2]
	
	double controlOut = 0;
	double controlOutOld = 0;
	
	//Konstanten wie in PID pdf
	double c0 = 0;
	double c1 = 0;
	double c2 = 0;
	
	double c0i = 0;
	double c1i = 0;
	double c2i = 0;
	
	double c0d = 0;
	double c1d = 0;
	double c2d = 0;
	
	double c0id = 0;
	double c1id = 0;
	double c2id = 0;

	
	public PID_Ver11(double desiredValue, double samplePeriod, double Kr, double Ki, double Kd, double AIWR, boolean ADWR) {
		this.desiredValue = desiredValue;
		this.samplePeriod = samplePeriod;
		
		this.Kr = Kr;
		this.Ki = Ki;
		this.Kd = Kd;
		
		this.AIWR = AIWR;
		
		// kein Windup/
		c0 = Kr*(1+(samplePeriod*Ki/2)+(Kd/samplePeriod));
		c1 = Kr*(-1+(samplePeriod*Ki/2)-2*(Kd/samplePeriod));
		c2 = Kr*(Kd/samplePeriod);
		
		// I-Windup-Reset
		c0i = Kr*(1+(Kd/samplePeriod));
		c1i = Kr*(-1-2*(Kd/samplePeriod));
		c2i = Kr*(Kd/samplePeriod);
		
		// D-Windup-Reset
		c0d = Kr*(1+(samplePeriod*Ki/2));
		c1d = Kr*(-1+(samplePeriod*Ki/2));
		c2d = 0;
		
		// I & D -Windup-Reset
		c0id = Kr;
		c1id = -1*Kr;
		c2id = 0;
		
	}
	
	public double runControl(double measuredValue) {	
		
		errorValue2 = errorValue1;
		errorValue1 = errorValue0;
		errorValue0 = desiredValue - measuredValue;
		controlOutOld = controlOut;
		
		// AIWR-Massnahme: noch nicht perfekt
		if(ADWR == false) {
			if(desiredValue - measuredValue > AIWR) {
				controlOut = c0i*errorValue0 + c1i*errorValue1 + c2i*errorValue2 + controlOutOld;	
			}
			else {
				controlOut = c0*errorValue0 + c1*errorValue1 + c2*errorValue2 + controlOutOld;
			}
		}
		else {
			if((desiredValue - measuredValue > AIWR) && (Math.signum(errorValue0) == Math.signum(errorValue2))) {
				controlOut = c0i*errorValue0 + c1i*errorValue1 + c2i*errorValue2 + controlOutOld;	
			}
			else if((desiredValue - measuredValue > AIWR) && (Math.signum(errorValue0) != Math.signum(errorValue2))) {
				controlOut = c0id*errorValue0 + c1id*errorValue1 + c2id*errorValue2 + controlOutOld;
			}
			else if((desiredValue - measuredValue < AIWR) && (Math.signum(errorValue0) == Math.signum(errorValue2))) {
				controlOut = c0*errorValue0 + c1*errorValue1 + c2*errorValue2 + controlOutOld;
			}
			else if((desiredValue - measuredValue < AIWR) && (Math.signum(errorValue0) != Math.signum(errorValue2))) {
				controlOut = c0d*errorValue0 + c1d*errorValue1 + c2d*errorValue2 + controlOutOld;
			}		
			
		}
		return controlOut;
	}
	
	public void updateDesiredValue(double desiredValue) {
		this.desiredValue = desiredValue;
	}
	
}