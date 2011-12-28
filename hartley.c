
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int discreteHartley( int Width, float* sample, float* outputSample ) {
	int n = 0;
	int N = Width; //SAMPLE WIDTH
	int k = 0;
	int result = 0;
	if ( N > 0 && sample && outputSample ) {
		for ( k = 0; N - 1 >= k; k++ ) {
			float outputValue = 0.0;
			for ( n = 0; N - 1 >= n; n++ ) {
				float coeff = ((2.0f * M_PI) / (float)N ) * (float)k * (float)n;
				float im = (sin(coeff) + cos(coeff));
				outputValue += sample[n] * im;
			}
			outputSample[k] = outputValue * ( 1.0f / sqrt( (float)N));
		}
		result = 0;
	} else {
		result = -1;
	}
	
	return result;
}

int testing() {
	int sampleWidth = 4;
	float testingSample[] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f
	};
	float* outputSample = (float*)malloc( sizeof(float) * sampleWidth );
	if ( discreteHartley( sampleWidth, testingSample, outputSample) == 0 ) {
		int i;
		for ( i = 0; sampleWidth - 1 >= i; i++ ) {
			printf("%f\n", outputSample[i]);
		}
		discreteHartley( sampleWidth, outputSample, testingSample );
		for ( i = 0; sampleWidth - 1 >= i; i++ ) {
			printf("%f\n", testingSample[i]); 
		}
	} else {
		printf("DHT failed\n");
	}
	
}

int main( int argn, char* argc[] ) {
	return testing();	
}
