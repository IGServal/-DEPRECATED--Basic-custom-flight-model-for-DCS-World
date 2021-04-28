//-----------------------------------------------------------------------------------------------
// These are utility functions defined by the University of Minnesota Flight Model Code
// They are mostly there to help look up tables.  I added in some limiter functions
//
// You could use this header to code up common math utilities your code may use multiple times
//-----------------------------------------------------------------------------------------------
#ifndef __UtilFunctions__
#define __UtilFunctions__

#include <malloc.h>
#include <memory.h>

// simple buffer helper
// reduce repeating malloc()/free()
class UtilBuffer
{
private: // disallow copying
	UtilBuffer(const UtilBuffer &other) {};
	UtilBuffer& operator=(const UtilBuffer &other) {return *this;};

public:
	double *vec; // array
	int capacity; // amount of elements in array
	//int used; // amount of elements in use

	UtilBuffer()
		: vec(NULL)
		, capacity(0)
	{}
	UtilBuffer(const int vertices)
		: vec(NULL)
		, capacity(0)
	{
		getVec(vertices);
	}
	~UtilBuffer() 
	{
		// cleanup on destroy (exiting scope)
		if (vec != NULL)
		{
			free(vec);
			vec = NULL;
		}
	}

	// support growing as needed
	double *getVec(const int vertices)
	{
		if (vertices > capacity)
		{
			free(vec);
			capacity = vertices;
			vec = (double*)malloc(vertices*sizeof(double));
		}
		return vec;
	}

	void copyVec(const int vertices, const double *src)
	{
		size_t toCopy = vertices*sizeof(double);
		double *dest = getVec(vertices); // check for size
		memcpy(dest, src, toCopy);
	}
};

// Start of Utility Functions

// Struct to define a set of data with a given number of dimenions and points
typedef struct {
		int nDimension;
		int *nPoints;   
		} ND_INFO;

// Matrix of all integers
int **intMatrix(int n,int m){
	int **mat = (int**) malloc(n*sizeof(int*));
	for(int i=0;i<n;i++)
		mat[i] = (int*) malloc(m*sizeof(int));
	return(mat);
	}

// Matrix of all doubles
double **doubleMatrix(int n,int m){
	double **mat = (double**) malloc(n*sizeof(double*));
	for(int i=0;i<n;i++)
		mat[i] = (double*) malloc(m*sizeof(double));
	return(mat);
	}

// Clear out the provided integer matrix
void freeIntMat(int **mat,int n,int m){
	/* the column size is not used but is required only
		for debugging purpose
	*/
	for(int i=0;i<n;i++)
		free(mat[i]);
	free(mat);
	}

// Clear out the providced double matrix
void freeDoubleMat(double **mat,int n,int m){
	/* the column size is not used but is required only
		for debugging purpose
	*/
	for(int i=0;i<n;i++)
		free(mat[i]);
	free(mat);
	}

// Error Call Helper Function
void ErrMsg(char *m){
	}

/************************************************/
/*    Get the indices of the hyper cube in the  */
/*    grid in which the point lies              */
/************************************************/
int **getHyperCube(double **Xmat, const double *V, const ND_INFO &ndinfo)
{
	int **indexMatrix = intMatrix(ndinfo.nDimension,2);
                     /*indexMatrix[i][0] => Lower, ...[1]=>Higher*/

	for(int i=0; i<ndinfo.nDimension; i++)
	{
		int indexMax = ndinfo.nPoints[i]; /* Get the total # of points in this dimension */
		double xmax = Xmat[i][indexMax-1];	 /* Get the upper bound along this axis */
		double xmin = Xmat[i][0];			/* Get the lower bound along this axis */

		/**************************************************************************** 
			It has been assumed that the gridpoints are monotonically increasing
			the zero index is the minimum and the max-1 is the maximum.
		*****************************************************************************/

		/****************************************************************************
        		Get the ith component in the vector V, the point at which we want to 
        		interpolate
		****************************************************************************/
		double x = V[i];

		/* Check to see if this point is within the bound */
		if(x<xmin || x>xmax)
		{
			ErrMsg("Point lies out data grid (in getHyperCube)");
		}
		else
		{
			for(int j=0; j<indexMax-1; j++)
			{
				if(x==Xmat[i][j])
				{
					indexMatrix[i][0] = indexMatrix[i][1] = j;
					break;
				}
				if(x==Xmat[i][j+1])
				{
					indexMatrix[i][0] = indexMatrix[i][1] = j+1;
					break;
				}
				if(x > Xmat[i][j] && x < Xmat[i][j+1] )
				{
					indexMatrix[i][0] = j;
					indexMatrix[i][1] = j+1;
					break;
				}
			}/*End of for(j=...) */
		}/*End of if-else */
	}/* End of for(i= ...) */

	return(indexMatrix);
} // getHyperCube()

/*********************************************************************************
 indexVector contains the co-ordinate of a point in the ndimensional grid
 the indices along each axis are assumed to begin from zero
 *********************************************************************************/
int getLinIndex(const int *indexVector, const ND_INFO &ndinfo)
{
	int linIndex=0;
	for(int i=0; i<ndinfo.nDimension; i++)
	{
		int P=1;
		for(int j=0; j<i; j++)
		{
			P = P*ndinfo.nPoints[j];
		}
		linIndex = linIndex + P*indexVector[i];
	}
	return(linIndex);
}

// Linearly interpolate between two data values
double linearInterpolate(const UtilBuffer &Tbuf, const double *V, double **Xmat, const ND_INFO &ndinfo)
{
	int *indexVector = (int*)malloc(ndinfo.nDimension * sizeof(int));
	int nVertices = 1<<(ndinfo.nDimension);

	UtilBuffer oldTbuf(nVertices);
	oldTbuf.copyVec(nVertices, Tbuf.vec);

	// reuse buffer until done here
	UtilBuffer newTbuf;

	int n = ndinfo.nDimension;
	int dimNum = 0;
	while(n>0)
	{
		int m = n-1;
		nVertices = (1<<m);

		newTbuf.getVec(nVertices); // prepare for next loop
		for(int i=0; i<nVertices; i++)
		{
			for(int j=0; j<m; j++)
			{
				int mask = (1<<j);
				indexVector[j] =  (mask & i) >> j;
			}/*End of for j*/

			int index1 = 0;
			int index2 = 0;
			for(int j=0; j<m; j++)
			{
				index1 = index1 + (1<<(j+1))*indexVector[j];
				index2 = index2 + (1<<j)*indexVector[j];
			}/*End of for j*/

			double f1 = oldTbuf.vec[index1];
			double f2 = oldTbuf.vec[index1+1];
			if(Xmat[dimNum][0] != Xmat[dimNum][1])
			{
				double lambda = (V[dimNum] - Xmat[dimNum][0]) / (Xmat[dimNum][1] - Xmat[dimNum][0]);

				newTbuf.vec[index2] = lambda*f2 + (1-lambda)*f1;
			}
			else
			{
				newTbuf.vec[index2] = f1;
			}
		}/*End of for i*/

		oldTbuf.copyVec(nVertices, newTbuf.vec);
		n=m;
		dimNum++;
	}/* End of while*/

	double result = oldTbuf.vec[0];
	free(indexVector);
	return(result);
} // linearInterpolate()

double interpn(double **Xmat, const double *Y, const double *xPar, const ND_INFO &ndinfo, UtilBuffer &Tbuf)
{
	const int nVertices = (1<<ndinfo.nDimension);

	int *indexVector = (int*)malloc(ndinfo.nDimension * sizeof(int));
	double **xPoint = doubleMatrix(ndinfo.nDimension,2);

	/* Get the indices of the hypercube containing the point in argument */
	int **indexMatrix = getHyperCube(Xmat, xPar, ndinfo);

	/* Get the co-ordinates of the hyper cube */
	for(int i=0; i<ndinfo.nDimension; i++)
	{
		int low  = indexMatrix[i][0];
		int high = indexMatrix[i][1];
		xPoint[i][0] = Xmat[i][low];
		xPoint[i][1] = Xmat[i][high];
	}

	for(int i=0; i<nVertices; i++)
	{
		for(int j=0; j<ndinfo.nDimension; j++)
		{
			int mask = 1<<j;
			int val = (mask & i) >> j;
			indexVector[j] = indexMatrix[j][val];
		}

		int index = getLinIndex(indexVector, ndinfo);
		Tbuf.vec[i] = Y[index];
	}

	double result = linearInterpolate(Tbuf, xPar, xPoint, ndinfo);

	free(indexVector);
	freeIntMat(indexMatrix, ndinfo.nDimension, 2);
	freeDoubleMat(xPoint, ndinfo.nDimension, 2);
	return(result);
}
// End of Utility Functions

// Simple upper and lower limiter
double limit(double input, double lower_limit, double upper_limit)
{
	if(input > upper_limit)
	{
		return upper_limit;
	}
	else if(input < lower_limit)
	{
		return lower_limit;
	}
	else
	{
		return input;
	}
}

#endif
