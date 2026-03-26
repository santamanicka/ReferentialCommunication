// ********************************************************
// A class for continuous-time recurrent neural networks
// with weighted sensor inputs rather than sensory neurons.
// ********************************************************

#include "SensorCTRNN.h"
#include "random.h"
#include <stdlib.h>
#include <iomanip>

using namespace std;

// A fast sigmoid implementation using a table w/ linear interpolation
#ifdef FAST_SIGMOID
int SigTableInitFlag = 0;
double SigTab[SigTabSize];

void InitSigmoidTable(void)
{
    if (!SigTableInitFlag) {
        double DeltaX = SigTabRange/(SigTabSize-1);
        for (int i = 0; i <= SigTabSize-1; i++)
          SigTab[i] = sigma(i * DeltaX);
        SigTableInitFlag = 1;
    }
}

double fastsigmoid(double x)
{
  if (x >= SigTabRange) return 1.0;
  if (x < 0) return 1.0 - fastsigmoid(-x);
  double id;
  double frac = modf(x*(SigTabSize-1)/SigTabRange, &id);
  int i = (int)id;
  double y1 = SigTab[i], y2 = SigTab[i+1];
 
  return y1 + (y2 - y1) * frac;
}
#endif

// ****************************
// Constructors and Destructors
// ****************************

// The constructor

SensorCTRNN::SensorCTRNN(int newcircuitsize, int newsensorsize)
{
    SetCircuitSize(newcircuitsize, newsensorsize);
    #ifdef FAST_SIGMOID
      InitSigmoidTable();
    #endif
}


// The destructor

SensorCTRNN::~SensorCTRNN()
{
    SetCircuitSize(0, 0);
}


// *********
// Utilities
// *********

// Resize a circuit.

void SensorCTRNN::SetCircuitSize(int newcircuitsize, int newsensorsize)
{
    circuitsize = newcircuitsize;
    sensorsize = newsensorsize;
    states.SetBounds(1,circuitsize);
    states.FillContents(0.0);
    outputs.SetBounds(1,circuitsize);
    outputs.FillContents(0.0);
    biases.SetBounds(1,circuitsize);
    biases.FillContents(0.0);
    gains.SetBounds(1,circuitsize);
    gains.FillContents(1.0);
    taus.SetBounds(1,circuitsize);
    taus.FillContents(1.0);
    Rtaus.SetBounds(1,circuitsize);
    Rtaus.FillContents(1.0);
    sensors.SetBounds(1, sensorsize);
    sensors.FillContents(0.0);
    circuitweights.SetBounds(1,circuitsize,1,circuitsize);
    circuitweights.FillContents(0.0);
    sensorweights.SetBounds(1, sensorsize, 1, circuitsize);
    sensorweights.FillContents(0.0);
}

// *******
// Control
// *******

// Randomize the states of a circuit.

void SensorCTRNN::RandomizeCircuit(double lb, double ub)
{
    for (int i = 1; i <= circuitsize; i++)
      SetNeuronState(i, UniformRandom(lb,ub));
}

void SensorCTRNN::RandomizeCircuit(double lb, double ub, RandomState &rs)
{
    for (int i = 1; i <= circuitsize; i++)
        SetNeuronState(i, rs.UniformRandom(lb,ub));
}

void SensorCTRNN::Initialize()
{
     for (int i = 1; i <= circuitsize; i++)
         SetNeuronState(i, 0);
     for (int i = 1; i <= sensorsize; i++)
        SetSensor(i, 0);
}

// ****************
// Input and Output
// ****************

ostream& operator<<(ostream& os, SensorCTRNN& c)
{
    // Set the precision
    os << setprecision(32);
    // Write the size
    os << c.sensorsize << " " << c.circuitsize << endl << endl;
    // Write the sensor weights
    for (int i = 1; i <= c.sensorsize; i++) {
        for (int j = 1; j <= c.circuitsize; j++)
            os << c.sensorweights[i][j] << " ";
        os << endl;
    }
    // Write the time constants
    for (int i = 1; i <= c.circuitsize; i++)
        os << c.taus[i] << " ";
    os << endl << endl;
    // Write the biases
    for (int i = 1; i <= c.circuitsize; i++)
        os << c.biases[i] << " ";
    os << endl << endl;
    // Write the gains
    for (int i = 1; i <= c.circuitsize; i++)
        os << c.gains[i] << " ";
    os << endl << endl;
    // Write the circuit weights
    for (int i = 1; i <= c.circuitsize; i++) {
        for (int j = 1; j <= c.circuitsize; j++)
            os << c.circuitweights[i][j] << " ";
        os << endl;
    }
    // Return the ostream
    return os;
}

istream& operator>>(istream& is, SensorCTRNN& c)
{
    // Read the size
    int circuitsize, sensorsize;
    is >> sensorsize;
    is >> circuitsize;
    c.SetCircuitSize(circuitsize, sensorsize);
    // Read the sensor weights
    for (int i = 1; i <= sensorsize; i++)
        for (int j = 1; j <= circuitsize; j++)
            is >> c.sensorweights[i][j];
    // Read the time constants
    for (int i = 1; i <= circuitsize; i++) {
        is >> c.taus[i];
        c.Rtaus[i] = 1/c.taus[i];
    }
    // Read the biases
    for (int i = 1; i <= circuitsize; i++)
        is >> c.biases[i];
    // Read the gains
    for (int i = 1; i <= circuitsize; i++)
        is >> c.gains[i];
    // Read the circuit weights
    for (int i = 1; i <= circuitsize; i++)
        for (int j = 1; j <= circuitsize; j++)
            is >> c.circuitweights[i][j];
    // Return the istream		
    return is;
}
		
