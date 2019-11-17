import java.util.Arrays;

public class hauptausführung {
	
//	public static void main(String[] args) {
//        System.out.println("Hello World!");
        
        public static void main(String[] args)
    	{
        	byte x1=35;
        	byte y1=0;
        	byte x2=5;
        	byte y2=-10;
        	
    		double[][] A = {
    				{ x1*x1*x1, x1*x1,  x1, 1},
    				{ x2*x2*x2, x2*x2, x2,  1},
    				{ 3*x1*x1,  2*x1, 1,  0},
    				{3*x2*x2,  2*x2, 1, 0}
    				};

    		double[] b = {y1, y2, 0, 0};

    		double[] x = LGS.solve(A, b);

    		System.out.println("x = " + Arrays.toString(x));
    	}
        
    }

