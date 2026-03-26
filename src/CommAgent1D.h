// *************************************
// A class for a 1-D agent that lives on
// a circle and has two angular distance
// sensors.
//
// PLW 12/15/08
// *************************************

#include "SensorCTRNN.h"
#include "globals.h"

#pragma once

#define NORMALIZE 1

#ifndef _PI_DEF
#define _PI_DEF
const double Pi = 3.141592654;
#endif

#ifdef FAST_TRIG
const int SinTabSize = 400;
const double SinTabRange = Pi/2.;

double fastsin(double x);
double fastcos(double x);
#endif

inline double sine(double x)
{
#ifndef FAST_TRIG
    return sin(x);
#else
    return fastsin(x);
#endif
}

inline double cosine(double x)
{
#ifndef FAST_TRIG
    return cos(x);
#else
    return fastcos(x);
#endif
}

// ****************************
// The CommAgent1D class declaration
// ****************************

class CommAgent1DCircle {
    public:
        // The constructor
        CommAgent1DCircle(double newMaxAngularVelocity, double newMaxSensorAngle,
                    int newCircuitSize, int newSensorSize);
        // The destructor
        ~CommAgent1DCircle() {};
        // Accessors
        double AngularPosition(void) {return angularPosition;};
        void SetAngularPosition(double newAngularPosition)
		{ //SM: basically the same as normalizeAngle(angle) method in main.cpp -- same as angle % (2*Pi)

            angularPosition = newAngularPosition - floor(newAngularPosition/(2*Pi))*2*Pi;
        }; 
        // Control
        void Update(CommAgent1DCircle &OtherAgent);
                void UpdateHomeVector();
        void UpdateTargetVector(double targetAngle);
        void Move(double stepsize);
        void MoveConstrained(double stepsize, double CounterClockwiseLimit,
                     double ClockwiseLimit);
        // IO; SM: friend funcs because they are actually implemented in SensorCTRNN. Here when we say Agent >>, then
	    // we mean to say Agent.CTRNN >>. That's why we just want to reuse the actual implementation of >> and <<
        friend ostream& operator<<(ostream& os, CommAgent1DCircle& a);
        friend istream& operator>>(istream& is, CommAgent1DCircle& a);

        double maxAngularVelocity, maxSensorAngle;
        double angularPosition, vAng; //SM: vAng = angular velocity
        SensorCTRNN NervousSystem;
        double CounterClockwiseForce, ClockwiseForce;
};

class CommAgent1DLine {
public:
	// The constructor
	CommAgent1DLine(double newLineLength, double newMaxLinearVelocity,
					double newMaxSensorDist, int newCircuitSize, int newSensorSize);
	// The destructor
	~CommAgent1DLine() {};
	// Accessors
	double LinearPosition(void) {return linearPosition;};
	void SetLinearPosition(double newLinearPosition, bool normalize)
	{ //SM: basically the same as normalizePosition(position) method in main.cpp -- same as position % lineLength
		
		linearPosition = newLinearPosition - floor(newLinearPosition/lineLength)*lineLength;
		//cout <<lineLength<< "In SetLinearPos, newLinearPosition = " << newLinearPosition << " linearPosition = " << linearPosition << endl;
	};
	void SetLinearPosition(double newLinearPosition)
	{ 
		
		linearPosition = newLinearPosition;
	};
	// Control
	void Update(CommAgent1DLine &OtherAgent);
	void UpdateHomeVector();
	void UpdateTargetVector(double targetPosition);
	void UpdateTargetVectorAddrOnly(double targetPosition);
	void Move(double stepsize);
	void MoveConstrained(double stepsize, double leftLimit, double rightLimit);
	// IO; SM: friend funcs because they are actually implemented in SensorCTRNN. Here when we say Agent >>, then
	// we mean to say Agent.CTRNN >>. That's why we just want to reuse the actual implementation of >> and <<
	friend ostream& operator<<(ostream& os, CommAgent1DLine& a);
	friend istream& operator>>(istream& is, CommAgent1DLine& a);
	
	double lineLength;
	double maxLinearVelocity, maxSensorDist;
	double linearPosition; //indicates agent's distance from its position to the "origin" on its *left*
	double vLinear;
	SensorCTRNN NervousSystem;
	double leftForce, rightForce;
};

