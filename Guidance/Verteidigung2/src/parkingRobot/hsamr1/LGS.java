package parkingRobot.hsamr1;

/**
 * /https://www.java-forum.org/thema/loesen-linearer-gleichungssysteme-mit-java.69267/
 */

public class LGS
{
	// Pivotvektor bestimmen
	static int[] pivot(double[][] A)
	{
		int n = A.length;
		int[] pivot = new int[n];
		for (int j = 0; j < n-1; j++)
		{
			double max = Math.abs(A[j][j]);
			int imax = j;
			for (int i = j+1; i < n; i++)
				if (Math.abs(A[i][j]) > max)
				{
					max  = Math.abs(A[i][j]);
					imax = i;
				}
			double[] h = A[j];
			A[j] = A[imax];
			A[imax] = h;
			pivot[j] = imax;
			for (int i = j+1; i < n; i++)
			{
				double f = -A[i][j]/A[j][j];
				for (int k = j+1; k < n; k++)
					A[i][k] += f*A[j][k];
				A[i][j] = -f;
			}
		}
		return pivot;
	}

	// loest das LGS Ax = b nach x auf
	public static double[] solve(double[][] A, double[] b)
	{
		double[][] B = A.clone();
		double[] x = b.clone();
		int[] pivot = pivot(B);
		int n = B.length;
		for (int i = 0; i < n-1; i++)
		{
			double h = b[pivot[i]];
			b[pivot[i]] = b[i];
			b[i] = h;
		}
		for (int j = 0; j < n; j++)
		{
			x[j] = b[j];
			for (int i = 0; i < j; i++)
				x[j] -= B[j][i]*x[i];
		}
		for (int j = n-1; j >= 0; j--)
		{
			for (int k = j+1; k < n; k++)
				x[j] -= B[j][k]*x[k];
			x[j] /= B[j][j];
		}
		return x;
	}
}