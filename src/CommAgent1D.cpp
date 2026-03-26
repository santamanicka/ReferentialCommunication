// *************************************
// A class for a 1-D agent that lives on
// a circle and has two angular distance
// sensors.
//
// PLW 12/15/08
// *************************************

#include "CommAgent1D.h"

using namespace std;

// A fast sigmoid implementation using a table w/ linear interpolation
#ifdef FAST_TRIG
int SinTableInitFlag = 0;
double SinTab[SinTabSize];

void InitSinTable(void)
{
    if (!SinTableInitFlag) {
        double DeltaX = SinTabRange/(SinTabSize-1);
        for (int i = 0; i <= SinTabSize-1; i++)
            SinTab[i] = sin(i * DeltaX);
        SinTableInitFlag = 1;
    }
}

double fastsin(double x)
{
    // normalize the angle to [0, 2*Pi)
    x -= floor(x/(2*Pi))*2*Pi;
    if (x > Pi) return -fastsin(x-Pi);
    if (x > Pi / 2.) return fastsin(Pi-x);
    double id;
    double frac = modf(x*(SinTabSize-1)/SinTabRange, &id);
    int i = (int)id;
    double y1 = SinTab[i], y2 = SinTab[i+1];
	
    return y1 + (y2 - y1) * frac;
}

double fastcos(double x)
{
    return fastsin(x+Pi/2.);
}

#endif

// *************************
// CommAgent1DCircle methods
// *************************

// The constructor

CommAgent1DCircle::CommAgent1DCircle(double newMaxAngularVelocity, double newMaxSensorAngle,
                               int newCircuitSize, int newSensorSize)
{	
    #ifdef FAST_TRIG
        InitSinTable();
    #endif
    // Initialize agent parameters and state
    maxAngularVelocity = newMaxAngularVelocity; maxSensorAngle = newMaxSensorAngle;
    angularPosition = 0; vAng = 0;
    NervousSystem.SetCircuitSize(newCircuitSize, newSensorSize);
}

// Update the agent's angular distance sensors with one other agent
void CommAgent1DCircle::Update(CommAgent1DCircle &OtherAgent) {
    double clockwiseDist, counterclockwiseDist, s;
    // the distance to the other agent, moving CW and CCW
    counterclockwiseDist = OtherAgent.AngularPosition() - angularPosition;
    clockwiseDist = -counterclockwiseDist;
    // normalize the distance to [0, 2*Pi) -- same as counterclockwiseDist % (2*Pi)
    counterclockwiseDist -= floor(counterclockwiseDist/(2*Pi))*2*Pi;
    clockwiseDist -= floor(clockwiseDist/(2*Pi))*2*Pi;
    // set the inputs for the CCW and CW distance sensors
    s = (maxSensorAngle - min(maxSensorAngle, counterclockwiseDist)) / maxSensorAngle;
    NervousSystem.SetSensor(1, s);
    s = (maxSensorAngle - min(maxSensorAngle, clockwiseDist)) / maxSensorAngle;
    NervousSystem.SetSensor(2, s);
}

// Update an agent's "home vector", which gives it information about it's
// current angular position
void CommAgent1DCircle::UpdateHomeVector() {
     NervousSystem.SetSensor(3, sine(angularPosition)/2 + 0.5);
     NervousSystem.SetSensor(4, cosine(angularPosition)/2 + 0.5);
}

// Update an agent's "target vector" sensors 
void CommAgent1DCircle::UpdateTargetVector(double targetAngle) {
     // Compute the angular difference between the agent and the target
     double bearingAngle = targetAngle - angularPosition;
     NervousSystem.SetSensor(3, sine(bearingAngle)/2 + 0.5); 
     NervousSystem.SetSensor(4, cosine(bearingAngle)/2 + 0.5); 
}

// update the agent's angular position
void CommAgent1DCircle::Move(double stepsize) {
     // Update the nervous system
    NervousSystem.EulerStep(stepsize);
    // Update the body effectors 
    CounterClockwiseForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize()-1);
    ClockwiseForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize());
    // Update the agent's angular position
    vAng = (CounterClockwiseForce - ClockwiseForce) * maxAngularVelocity;
    SetAngularPosition(angularPosition + stepsize * vAng);
}     

// update the agent's angular position, constraining it to lie within the
// specified range (the limits should be between 0 and 2*Pi)
void CommAgent1DCircle::MoveConstrained(double stepsize,
                     double CounterClockwiseLimit, double ClockwiseLimit) {
    double newPos;
    // Update the nervous system
    NervousSystem.EulerStep(stepsize);
    // Update the body effectors 
    CounterClockwiseForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize()-1);
    ClockwiseForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize());
    // Update the agent's angular position
    vAng = (CounterClockwiseForce - ClockwiseForce) * maxAngularVelocity;
    newPos = angularPosition + stepsize * vAng;
    if (newPos > CounterClockwiseLimit &&
        angularPosition <= CounterClockwiseLimit)
        newPos = CounterClockwiseLimit;
    else if (newPos < ClockwiseLimit && angularPosition >= ClockwiseLimit)
        newPos = ClockwiseLimit;
    SetAngularPosition(newPos);
}

// ***********************
// CommAgent1DLine methods
// ***********************

// The constructor

CommAgent1DLine::CommAgent1DLine(double newLineLength, double newMaxLinearVelocity,
								 double newMaxSensorDist, int newCircuitSize, int newSensorSize)
{	
    // Initialize agent parameters and state
	lineLength = newLineLength;
    maxLinearVelocity = newMaxLinearVelocity; maxSensorDist = newMaxSensorDist;
    linearPosition = 0.0; vLinear = 0.0;
    NervousSystem.SetCircuitSize(newCircuitSize, newSensorSize);
}

// Update the agent's distance sensors with one other agent
void CommAgent1DLine::Update(CommAgent1DLine &OtherAgent) {
    double leftDist, rightDist, s;
    // the distance to the other agent, moving left and right
    leftDist = linearPosition - OtherAgent.LinearPosition();
	// normalize the distance to [0, lineLength) -- equivalent to a modulus operation
    leftDist -= floor(leftDist/lineLength)*lineLength;
    rightDist = lineLength - leftDist;
	rightDist = (rightDist == lineLength ? 0.0 : rightDist);
    // set the inputs for the left and right distance sensors
    s = (maxSensorDist - min(maxSensorDist, leftDist)) / maxSensorDist;
    NervousSystem.SetSensor(1, s);
    s = (maxSensorDist - min(maxSensorDist, rightDist)) / maxSensorDist;
    NervousSystem.SetSensor(2, s);
	//cout << "Mylpos = " << linearPosition << " OtherlPos = " << OtherAgent.linearPosition <<
	//" Leftdist = " << leftDist << " Rightdist = " << rightDist << endl;
}

// Update an agent's "home vector", which gives it information about it's current
// linear position. NOTE: 'linearPosition' means left coordinate by definition.
void CommAgent1DLine::UpdateHomeVector() {
	double leftCoordinate = linearPosition;
	double rightCoordinate = lineLength - linearPosition;
	rightCoordinate = (rightCoordinate == lineLength ? 0.0 : rightCoordinate);
	NervousSystem.SetSensor(3, leftCoordinate/lineLength);
#if !SINGLE_BEARING_SENSOR
	NervousSystem.SetSensor(4, rightCoordinate/lineLength);
#endif
	//cout << "LinearPos = " << linearPosition << " LeftHcoord = " << leftCoordinate << " RightHcoord = " << rightCoordinate << endl;
}

// Update an agent's "target vector" sensors 
void CommAgent1DLine::UpdateTargetVector(double targetPosition) {
	// Compute the angular difference between the agent and the target
	double leftDistance = linearPosition - targetPosition;
	leftDistance -= floor(leftDistance/lineLength)*lineLength;
	double rightDistance = lineLength - leftDistance;
	rightDistance = (rightDistance == lineLength ? 0.0 : rightDistance);
	//cout << targetPosition << linearPosition << distToTarget << leftDistance << rightDistance << endl;
	NervousSystem.SetSensor(3, leftDistance/lineLength); 
#if !SINGLE_BEARING_SENSOR
	NervousSystem.SetSensor(4, rightDistance/lineLength); 
#endif
	//cout << "TargetPos = " << targetPosition << " LinearPos = " << linearPosition <<
	//" LeftTcoord = " << leftDistance << " RightTcoord = " << rightDistance << endl;
}

// Update an agent's "target vector" sensors with only the target's "Address"
void CommAgent1DLine::UpdateTargetVectorAddrOnly(double targetPosition) {
	NervousSystem.SetSensor(3, targetPosition/WorldLength);
	double rTargetPosition = WorldLength - targetPosition;
	rTargetPosition = (rTargetPosition == WorldLength ? 0.0 : rTargetPosition);
#if !SINGLE_BEARING_SENSOR
	NervousSystem.SetSensor(4, rTargetPosition/WorldLength);
#endif
	//cout << "LinearPos = " << linearPosition << " LeftHcoord = " << leftCoordinate << " RightHcoord = " << rightCoordinate << endl;
}

// update the agent's linear position
void CommAgent1DLine::Move(double stepsize) {
	// Update the nervous system
	//cout << "In move" << endl;
    NervousSystem.EulerStep(stepsize);
	//cout << "Euler stepped once" << endl;
    // Update the body effectors 
    leftForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize()-1);
    rightForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize());
    // Update the agent's linear position. Moving right = increasing linearPosition
    vLinear = (rightForce - leftForce) * maxLinearVelocity;
	double newLinearPosition = linearPosition + stepsize * vLinear; 
	//cout << linearPosition << vLinear << endl;
	//cout << "New linear position = " << newLinearPosition << endl;
    SetLinearPosition(newLinearPosition, NORMALIZE);
}     

// update the agent's linear position, constraining it to lie within the
// specified range (the limits should be between 0 and lineLength)
void CommAgent1DLine::MoveConstrained(double stepsize,
										double leftLimit, double rightLimit) {
    double newPos;
    // Update the nervous system
    NervousSystem.EulerStep(stepsize);
    // Update the body effectors 
    leftForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize()-1);
    rightForce = NervousSystem.NeuronOutput(NervousSystem.CircuitSize());
    // Update the agent's angular position
    vLinear = (rightForce - leftForce) * maxLinearVelocity;
    newPos = linearPosition + stepsize * vLinear;
	newPos -= floor(newPos/lineLength)*lineLength;
	if(leftLimit < rightLimit)
	{
		if(newPos < leftLimit || newPos > rightLimit)
		{
			if(vLinear < 0) newPos = leftLimit;
			else if(vLinear > 0) newPos = rightLimit;
		}
	}
	else if(rightLimit < leftLimit)
	{
		if (newPos > rightLimit && newPos < leftLimit)
		{
			if(vLinear > 0) newPos = rightLimit;
			else if(vLinear < 0) newPos = leftLimit;
		}
	}
	if(linearPosition != newPos) 
		SetLinearPosition(newPos);
	//cout << "LL, RL = " << leftLimit << "," << rightLimit << " Newpos = " << newPos << endl;
}

// ****************
// Input and Output
// ****************

ostream& operator<<(ostream& os, AGENT& a)
{
    // Write the nervous system
    os << a.NervousSystem << endl;
    // Return the ostream
    return os;
}

istream& operator>>(istream& is, AGENT& a)
{
    // Read the nervous system
    is >> a.NervousSystem;
    // Return the istream		
    return is;
}
