package parkingRobot.hsamr1;

public class PID_Ver2 {
	
	double desiredValue = 0;
	double samplePeriod = 0;
	
	double KP = 0;
	double KI = 0;
	double KD = 0;
	double AIWR = 99999999;	//Anti-I-Reset-Windup
	boolean ADWR = false;	//Anti-D-Reset-Windup	
	
	double errorValue0 = 0; 
	double errorValue1 = 0;	//e[k-1]
	double errorValue2 = 0; //e[k-2]
	
	double controlOut = 0;
	double controlOutOld = 0;
	
	// constants like documentation
	double c0 = 0;
	double c1 = 0;
	double c2 = 0;
	
	double c0I = 0;
	double c1I = 0;
	double c2I = 0;
	
	double c0D = 0;
	double c1D = 0;
	double c2D = 0;
	
	double c0ID = 0;
	double c1ID = 0;
	double c2ID = 0;

	
	public PID_Ver2(double desiredValue, double samplePeriod, double KP, double KI, double KD, double AIWR, boolean ADWR) {
		this.desiredValue = desiredValue;
		this.samplePeriod = samplePeriod;
		
		this.KP = KP;
		this.KI = KI;
		this.KD = KD;
		
		this.AIWR = AIWR;
		
		// no Windup/
		c0 = KP*(1+(samplePeriod*KI/2)+(KD/samplePeriod));
		c1 = KP*(-1+(samplePeriod*KI/2)-2*(KD/samplePeriod));
		c2 = KP*(KD/samplePeriod);
		
		// I-Windup-Reset
		c0I = KP*(1+(KD/samplePeriod));
		c1I = KP*(-1-2*(KD/samplePeriod));
		c2I = KP*(KD/samplePeriod);
		
		// D-Windup-Reset
		c0D = KP*(1+(samplePeriod*KI/2));
		c1D = KP*(-1+(samplePeriod*KI/2));
		c2D = 0;
		
		// I & D -Windup-Reset
		c0ID = KP;
		c1ID = -1*KP;
		c2ID = 0;
		
	}
	
	public double runControl(double measuredValue) {	
		
		errorValue2 = errorValue1;
		errorValue1 = errorValue0;
		errorValue0 = desiredValue - measuredValue;
		controlOutOld = controlOut;
		
		// checking for AWR
		if(ADWR == false) {
			if(desiredValue - measuredValue > AIWR) {
				controlOut = c0I*errorValue0 + c1I*errorValue1 + c2I*errorValue2 + controlOutOld;	
			}
			else {
				controlOut = c0*errorValue0 + c1*errorValue1 + c2*errorValue2 + controlOutOld;
			}
		}
		else {
			if((desiredValue - measuredValue > AIWR) && (Math.signum(errorValue0) == Math.signum(errorValue2))) {
				controlOut = c0I*errorValue0 + c1I*errorValue1 + c2I*errorValue2 + controlOutOld;	
			}
			else if((desiredValue - measuredValue > AIWR) && (Math.signum(errorValue0) != Math.signum(errorValue2))) {
				controlOut = c0ID*errorValue0 + c1ID*errorValue1 + c2ID*errorValue2 + controlOutOld;
			}
			else if((desiredValue - measuredValue < AIWR) && (Math.signum(errorValue0) == Math.signum(errorValue2))) {
				controlOut = c0*errorValue0 + c1*errorValue1 + c2*errorValue2 + controlOutOld;
			}
			else if((desiredValue - measuredValue < AIWR) && (Math.signum(errorValue0) != Math.signum(errorValue2))) {
				controlOut = c0D*errorValue0 + c1D*errorValue1 + c2D*errorValue2 + controlOutOld;
			}		
			
		}
		return controlOut;
	}
	
	public void updateDesiredValue(double desiredValue) {
		this.desiredValue = desiredValue;
	}
	
}