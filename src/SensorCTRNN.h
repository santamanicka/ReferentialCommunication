// *****************************************************
// A class for continuous-time recurrent neural networks
// with no sensory neurons
// RDB--modified by PLW 
//  8/94 Created
//  12/98 Optimized integration
//  1/08 Added table-based fast sigmoid w/ linear interpolation
// *****************************************************

// Uncomment the following line for table-based fast sigmoid w/ linear interpolation
#define FAST_SIGMOID

#include "VectorMatrix.h"
#include "random.h"
#include <iostream>
#include <math.h>

using namespace std;

#pragma once

#ifdef FAST_SIGMOID
const int SigTabSize = 400;
const double SigTabRange = 15.0;

double fastsigmoid(double x);
#endif

inline double sigma(double x) 
{
  return 1/(1 + exp(-x));
}

inline double sigmoid(double x)
{
#ifndef FAST_SIGMOID
  return sigma(x);
#else
  return fastsigmoid(x);
#endif
}

// The inverse sigmoid function

inline double InverseSigmoid(double y)
{
  return log(y/(1-y));
}

// The SensorCTRNN class declaration

class SensorCTRNN {
    public:
        // The constructor
        SensorCTRNN(int newcircuitsize = 0, int newsensorsize = 0);
        // The destructor
        ~SensorCTRNN();
        
        // Accessors
        int CircuitSize(void) {return circuitsize;};
		int SensorSize(void) {return sensorsize;};
        void SetCircuitSize(int newcircuitsize, int newsensorsize);
        double NeuronState(int i) {return states[i];};
        void SetNeuronState(int i, double value) 
            {states[i] = value;outputs[i] = sigmoid(gains[i]*(states[i] + biases[i]));};
        double NeuronOutput(int i) {return outputs[i];};
        void SetNeuronOutput(int i, double value) 
            {outputs[i] = value; states[i] = InverseSigmoid(value)/gains[i] - biases[i];};
		double Sensor(int i) {return sensors[i];};
		void SetSensor(int i, double value) {sensors[i] = value;};
        double NeuronBias(int i) {return biases[i];};
        void SetNeuronBias(int i, double value) {biases[i] = value;};
        double NeuronGain(int i) {return gains[i];};
        void SetNeuronGain(int i, double value) {gains[i] = value;};
        double NeuronTimeConstant(int i) {return taus[i];};
        void SetNeuronTimeConstant(int i, double value) {taus[i] = value;Rtaus[i] = 1/value;};
        double CircuitWeight(int from, int to) {return circuitweights[from][to];};
        void SetCircuitWeight(int from, int to, double value) {circuitweights[from][to] = value;};
		double SensorWeight(int from, int to) {return sensorweights[from][to];};
		void SetSensorWeight(int from, int to, double value) {sensorweights[from][to] = value;};
        void LesionNeuron(int n) 
        {
            for (int i = 1; i<= circuitsize; i++) {
                SetCircuitWeight(i,n,0);
                SetCircuitWeight(n,i,0);
            }
        }

        // Input and output
        friend ostream& operator<<(ostream& os, SensorCTRNN& c);
        friend istream& operator>>(istream& is, SensorCTRNN& c);
        
        // Control
        void RandomizeCircuit(double lb, double ub);
        void RandomizeCircuit(double lb, double ub, RandomState &rs);
	void Initialize();
        void EulerStep(double stepsize)
        {
            // Update the state of all neurons.
            for (int i = 1; i <= circuitsize; i++) {
                double input = 0;
				for (int j = 1; j <= sensorsize; j++) { 
                    input += sensorweights[j][i] * sensors[j];
                }
                for (int j = 1; j <= circuitsize; j++) { 
                    input += circuitweights[j][i] * outputs[j];
                }
                states[i] += stepsize * Rtaus[i] * (input - states[i]);
            }  
            // Update the outputs of all neurons.
            for (int i = 1; i <= circuitsize; i++) {      
                outputs[i] = sigmoid(gains[i] * (states[i] + biases[i]));
            }
        };
		
        // Control
        int circuitsize, sensorsize;
        TVector<double> states, outputs, biases, gains, taus, Rtaus, sensors;
        TMatrix<double> circuitweights, sensorweights;
        TVector<double> TempStates,TempOutputs,k1,k2,k3,k4;
};

