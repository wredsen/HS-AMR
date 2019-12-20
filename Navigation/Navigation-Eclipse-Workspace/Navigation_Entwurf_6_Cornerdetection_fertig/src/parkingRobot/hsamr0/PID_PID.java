package parkingRobot.hsamr0;

public class PID_PID {
	
	double desiredValue = 0;
	double samplePeriod = 0;
	
	double Kr = 0;
	double Ki = 0;
	double Kd = 0;
	
	double errorValue0 = 0; 
	double errorValue1 = 0;	//e[k-1]
	double errorValue2 = 0; //e[k-2]
	
	double controlOut = 0;
	double controlOutOld = 0;
	
	//Konstanten wie in PID pdf
	double c0 = 0;
	double c1 = 0;
	double c2 = 0;

	
	public PID_PID(double desiredValue, double samplePeriod, double Kr, double Ki, double Kd) {
		this.desiredValue = desiredValue;
		this.samplePeriod = samplePeriod;
		
		this.Kr = Kr;
		this.Ki = Ki;
		this.Kd = Kd;
		
		c0 = Kr*(1+(samplePeriod*Ki/2)+(Kd/samplePeriod));
		c1 = Kr*(-1+(samplePeriod*Ki/2)-2*(Kd/samplePeriod));
		c2 = Kr*(Kd/samplePeriod);
	}
	
	public double runControl(double measuredValue) {	
		errorValue2 = errorValue1;
		errorValue1 = errorValue0;
		errorValue0 = desiredValue - measuredValue;
		controlOutOld = controlOut;
		
		controlOut = c0*errorValue0 + c1*errorValue1 + c2*errorValue2 + controlOutOld;		
		return controlOut;
	}
	
}