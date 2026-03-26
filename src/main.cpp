#include <cstdlib>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <cmath>
#include "Search.h"
#include "SensorLoadCircuit.h"
#include "CommAgent1D.h"
#include "globals.h"

using namespace std;

void InitializeSearchTrial(AGENT &Sender, AGENT &Receiver);
double RunAgents(AGENT &Sender, AGENT &Receiver);
double RunTrial(AGENT &Sender, AGENT &Receiver, double target);
void StepBothAgents(AGENT &Sender, AGENT &Receiver, double target);
void StepReceiverOnly(AGENT &Receiver);
void StepSenderOnly(AGENT &Sender);
void LoadCircuitParameters(SensorCTRNN &SenderNervousSystem,
                           SensorCTRNN &ReceiverNervousSystem, TVector<double> &v);
double EvaluateAgents(TVector<double> &v, RandomState &);
double EvaluateAgents(TVector<double> &v);
void DumpBestCircuit(int generation, TVector<double> &v);
void DumpFinalCircuits(int generation, TVector<double> &v);
void DisplaySearchResults(TSearch &s);
#ifdef CIRCLE
void PrepareCircleWorld();
// normalize an angle to [0, 2*Pi)
double normalizeAngle(double angle);
// the separation between two angles, normalized to [0, Pi]
double angleSeparation(double angle1, double angle2);
#endif
#ifdef LINE
void PrepareLineWorld();
// normalize position to [0, lineLength)
double normalizePosition(double position);
double linearSeparation(double position1, double position2);
#endif
#ifdef TRACK_LINEAGES
void DisplayLineage(TSearch &s, treeNode node, ofstream &ofs);
#endif

//Globals
#ifdef CIRCLE
// settings for the evaluation trials for world = CIRCLE
TVector <double> initSenderAngles, initAgentSeparations, targetAngles;
double CCWLimit, CWLimit;
#endif
#ifdef LINE
// settings for the evaluation trials for world = LINE
TVector <double> initSenderPositions, initAgentSeparations, targetPositions;
TVector <double> leftLimits, rightLimits;
double currentLeftLimit, currentRightLimit;
#endif
int NumTrials;
const char *senderresultfname = "bestSender.ns";
const char *receiverresultfname = "bestReceiver.ns";
#ifdef TRACK_LINEAGES
string lineagefolder = "lineage";
string lineageresultfname = lineagefolder + "/bestLineage.out";
#endif

long RandomSeed;

int main(int argc, char *argv[]) {   
    // get the random seed from the command line
    if (argc != 2) {
        cerr << "Usage: *.exe RandomSeed\n";
        return 0;
    }
    else {
        RandomSeed = atol(argv[1]);
    }
    
	#ifdef CIRCLE
	PrepareCircleWorld();
	#endif
	
	#ifdef LINE
	PrepareLineWorld();
	//cout << "Line prepared" << endl;
	#endif

	/*
	AGENT Sender(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
    AGENT Receiver(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
	ifstream in1(senderresultfname);
	in1 >> Sender;
	ifstream in2(receiverresultfname);
	in2 >> Receiver;
	
	cout << Sender << endl;
	cout << Receiver << endl;
	
	cout << RunAgents(Sender, Receiver) << endl;
	*/
	
    // Set up and run the search
    TSearch s(NumParameters); 
    s.SetRandomSeed(RandomSeed);
    s.SetEvaluationFunction(EvaluateAgents);
    s.SetSelectionMode(SelectionMode);
    s.SetReproductionMode(ReproductionMode);
#if GA_MODE
	s.SetElitistFraction(ElitistFraction);
#endif
    s.SetPopulationSize(PopulationSize); 
    s.SetMaxGenerations(MaxGenerations);
    s.SetMutationVariance(MutationVariance);
    s.SetBestActionFunction(DumpBestCircuit);
    s.SetPopulationStatisticsDisplayFunction(NULL);
    s.SetSearchResultsDisplayFunction(NULL);
    s.SetMaxExpectedOffspring(FitnessScalingMultiple);
    s.SetSearchFold(SearchFoldFlag);
    s.SetSearchConstraint(SearchConstrainedFlag);
    s.SetReEvaluationFlag(ReEvaluationFlag);
    s.SetCheckpointInterval(CheckPointInterval);
    //s.SetSearchResultsDisplayFunction(DisplaySearchResults);
    s.ExecuteSearch();
	//s.ResumeSearch();
	    
    return EXIT_SUCCESS;
}

void InitializeSearchTrial(AGENT &Sender, AGENT &Receiver) {
    Sender.NervousSystem.Initialize();
    Receiver.NervousSystem.Initialize();
}

double EvaluateAgents(TVector<double> &v, RandomState &) {
    return EvaluateAgents(v);
}

double EvaluateAgents(TVector<double> &v) {
#ifdef CIRCLE
	AGENT Sender(MaxAngularVelocity, MaxSensorAngle, CIRCUITSIZE, SENSORSIZE);
    AGENT Receiver(MaxAngularVelocity, MaxSensorAngle, CIRCUITSIZE, SENSORSIZE);
#endif
#ifdef LINE
	AGENT Sender(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
    AGENT Receiver(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
#endif
	//cout << "Agents created" << endl;
	//cout << "Receiver's linear position = " << Receiver.linearPosition << endl;
    LoadCircuitParameters(Sender.NervousSystem, Receiver.NervousSystem, v);
	//cout << "Circuit loaded" << endl;
    return RUNAGENTFUNCTION(Sender, Receiver);
}

#ifdef LINE //All LINE methods --------------------------------------------------------------
void PrepareLineWorld() {
/*
#if CONSTRAIN_SENDER
	// set the left and right limits on the sender's motion with pos *centered at 0*
	leftLimit = normalizePosition(-SenderRegion/2);
	rightLimit = normalizePosition(SenderRegion/2);
#endif
*/
    // assign settings for the evaluation trials
	
    // initial positions for the sender 
    initSenderPositions.SetSize(NumInitSenderPositionsInZone*NumConstraintZones);
	leftLimits.SetSize(NumInitSenderPositionsInZone*NumConstraintZones);
	rightLimits.SetSize(NumInitSenderPositionsInZone*NumConstraintZones);
#if CONSTRAIN_SENDER
    // if the sender is constrained, distribute the starting positions
    // within the constrained region
	for (int zone = 1; zone <= NumConstraintZones; zone++)
	{
		double zoneCenter = (zone-1)*(WorldLength/2.)/NumConstraintZones;
		if (NumConstraintZones > 1) {
			zoneCenter -= SenderRegion/2.; //[8.75, 1.25]
		}
		double leftLimit = normalizePosition(zoneCenter - (SenderRegion/2));
		double rightLimit = normalizePosition(zoneCenter + (SenderRegion/2));
		for (int i = 1; i <= NumInitSenderPositionsInZone; i++)
		{
			double pos = leftLimit + i*SenderRegion/(NumInitSenderPositionsInZone+1);
			if(pos < 0.0 || pos >= WorldLength)
				pos = normalizePosition(pos);
			initSenderPositions[((zone-1)*NumInitSenderPositionsInZone)+i] = pos;
			leftLimits[((zone-1)*NumInitSenderPositionsInZone)+i] = leftLimit;
			rightLimits[((zone-1)*NumInitSenderPositionsInZone)+i] = rightLimit;
		}
	}
#else
    // otherwise, distribute the positions uniformly around the arena
    for (int i = 1; i <= NumInitSenderPositions; i++)
	{
		double pos = (i-1)*WorldLength/NumInitSenderPositions;
		if(pos < 0.0 || pos >= WorldLength)
			pos = normalizePosition(pos);
		initSenderPositions[i] = pos;
	}
#endif
	
	cout << "Sender Positions" << endl;
	cout << initSenderPositions << endl;
	
	cout << "Left and right limits" << endl;
	cout << leftLimits << endl;
	cout << rightLimits << endl;

    // initial separations between the sender and receiver
	initAgentSeparations.SetSize(NumInitAgentSeparations);
	for (int i = 1; i <= NumInitAgentSeparations; i++) //sep ranges from -Max to +Max
	{
		initAgentSeparations[i] = -MaxInitSeparation + (i-1)*2*MaxInitSeparation/(NumInitAgentSeparations-1);
		//cout << MaxInitSeparation << " " << initAgentSeparations[i] << endl;
	}
	//SM: 2*MaxInitSeparation is to spread the separations on either sides of the agents
	
	cout << "Agent Separations" << endl;
	cout << initAgentSeparations << endl;
    
    // target locations
    targetPositions.SetSize(NumTargetPositions);
#if CONSTRAIN_SENDER
    // if the sender is constrained, distribute the targets on the "opposite
    // half" of the line
	double zoneCenter,senderLeftMostLimit,senderRightMostLimit;
	int zone;
	if (NumConstraintZones > 1) {
		zone = 1;
		zoneCenter = (zone - 1)*(WorldLength/2.)/NumConstraintZones;
		zoneCenter -= SenderRegion/2.; //8.75
		senderLeftMostLimit = normalizePosition(zoneCenter - (SenderRegion/2)); //7.5
		zone = 2;
		zoneCenter = (zone - 1)*(WorldLength/2.)/NumConstraintZones;
		zoneCenter -= SenderRegion/2.; //1.25
		senderRightMostLimit = normalizePosition(zoneCenter + (SenderRegion/2)); //2.5
	}
	else {
		zone = 1;
		zoneCenter = (zone - 1)*(WorldLength/2.)/NumConstraintZones;
		senderLeftMostLimit = normalizePosition(zoneCenter - (SenderRegion/2));
		senderRightMostLimit = normalizePosition(zoneCenter + (SenderRegion/2));
	}
	double targetRangeStart = senderRightMostLimit + ZoneTargetRangeSeparation;
	double targetRangeEnd = senderLeftMostLimit - ZoneTargetRangeSeparation;
	double targetZoneLength = targetRangeEnd - targetRangeStart;
	for (int i = 1; i <= NumTargetPositions; i++)
	{
		double pos = targetRangeStart + ((i-1)*targetZoneLength/(NumTargetPositions-1));  
		if(pos < 0.0 || pos >= WorldLength)
			pos = normalizePosition(pos);
		targetPositions[i] = pos;
	}
#else
    // otherwise, distribute the targets uniformly around the arena
    for (int i = 1; i <= NumTargetPositions; i++)
	{
        double pos = (i-1)*WorldLength/NumTargetPositions; 
		if(pos < 0.0 || pos >= WorldLength)
			pos = normalizePosition(pos);
		targetPositions[i] = pos;
	}
#endif
	
	cout << "Target Positions" << endl;
	cout << targetPositions << endl;
	
    NumTrials = initSenderPositions.Size() * initAgentSeparations.Size() * targetPositions.Size();
}

double RunAgents(CommAgent1DLine &Sender, CommAgent1DLine &Receiver) {
	//cout << "In RunAgents, Receiver's linear position = " << Receiver.linearPosition << endl;
    double Total = 0;
	for (int zone = 1; zone <= NumConstraintZones; zone++) {
		for (int senderPosIndex = 1; senderPosIndex <= NumInitSenderPositionsInZone; senderPosIndex++) {
			currentLeftLimit = leftLimits[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex];
			currentRightLimit = rightLimits[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex];
			for (int agentSepIndex = 1; agentSepIndex <= initAgentSeparations.Size(); agentSepIndex++) {
				for (int targetPosIndex = 1; targetPosIndex <= NumTargetPositions; targetPosIndex++) {
					Sender.SetLinearPosition(initSenderPositions[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex]);
					//cout << "In RunAgents loop, Sender's linear position = " << Sender.linearPosition << endl;
					Receiver.SetLinearPosition(initSenderPositions[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex] + 
											   initAgentSeparations[agentSepIndex], NORMALIZE);
					//cout << "In RunAgents loop, Receiver's linear position = " << Receiver.linearPosition << endl;
					Total += RunTrial(Sender, Receiver, targetPositions[targetPosIndex]);
					//cout << "Trial run once" << endl;
				}
			}
		}
	}
    return Total / NumTrials;
}

double RunTrial(CommAgent1DLine &Sender, CommAgent1DLine &Receiver, double targetPosition) {	
	//cout << "Running trial" << endl;
	//cout << "In RunTrial, Receiver's linear position = " << Receiver.linearPosition << endl;
    double diff1, diff2;
    INITTRIALFUNCTION(Sender, Receiver);
#if ONE_INTERACTION_ONLY
	double time = 0, agentSep = linearSeparation(Sender.LinearPosition(),
												Receiver.LinearPosition());
	while (time < TrialDuration && agentSep <= MaxSensorDist) {
		StepBothAgents(Sender, Receiver, targetPosition); 
		agentSep = linearSeparation(Sender.LinearPosition(), Receiver.LinearPosition());
		time += StepSize;
	}
	Receiver.Update(Sender);
	while (time < TrialDuration) {
		StepReceiverOnly(Receiver);
		time += StepSize;
	}
	diff1 = linearSeparation(Receiver.LinearPosition(), targetPosition);  
	time = 0, agentSep = linearSeparation(Sender.LinearPosition(),
										 Receiver.LinearPosition());
	while (time < PostTrialInterval && agentSep <= MaxSensorDist) {
		StepBothAgents(Sender, Receiver, targetPosition);
		agentSep = linearSeparation(Sender.LinearPosition(), Receiver.LinearPosition());
		time += StepSize;
	}
	Receiver.Update(Sender);
	while (time < PostTrialInterval) {
		StepReceiverOnly(Receiver);
		time += StepSize;
	}
	diff2 = linearSeparation(Receiver.LinearPosition(), targetPosition);
#elif CLAMP_RECEIVER
#if TARGET_ADDRESS_ONLY
	Sender.UpdateTargetVectorAddrOnly(targetPosition);
#endif
	for (double time = 0; time < InfoTransmissionInterval; time += StepSize)
	{
		StepSenderOnly(Sender);
#if !TARGET_ADDRESS_ONLY
		Sender.UpdateTargetVector(targetPosition);
#endif
		Sender.Update(Receiver);
		Receiver.Update(Sender);
		Receiver.NervousSystem.EulerStep(StepSize);
	}
	//Sender disappers, so set receiver's angular sensors to zero
	Receiver.NervousSystem.SetSensor(1, 0.0);
	Receiver.NervousSystem.SetSensor(2, 0.0);
	for (double time = 0; time < TrialDuration; time += StepSize)
		StepReceiverOnly(Receiver);
	diff1 = linearSeparation(Receiver.LinearPosition(), targetPosition);
	for (double time = 0; time < PostTrialInterval; time += StepSize)
		StepReceiverOnly(Receiver);
	diff2 = linearSeparation(Receiver.LinearPosition(), targetPosition);
#else
#if TARGET_ADDRESS_ONLY
	Sender.UpdateTargetVectorAddrOnly(targetPosition);
#endif
	for (double time = 0; time < TrialDuration; time += StepSize)
		StepBothAgents(Sender, Receiver, targetPosition);
	diff1 = linearSeparation(Receiver.LinearPosition(), targetPosition);
	for (double time = 0; time < PostTrialInterval; time += StepSize)
		StepBothAgents(Sender, Receiver, targetPosition);
	diff2 = linearSeparation(Receiver.LinearPosition(), targetPosition);
#endif
	//cout << diff1 << diff2 << endl;
    return (1.0 - ((diff1 + diff2) / WorldLength));
}

void StepBothAgents(CommAgent1DLine &Sender, CommAgent1DLine &Receiver, double targetPosition) {
	//cout << "In StepBoth, Receiver's linear position = " << Receiver.linearPosition << endl;
#if CONSTRAIN_SENDER
	Sender.MoveConstrained(StepSize, currentLeftLimit, currentRightLimit);
#else
	Sender.Move(StepSize);
#endif
	//cout << "Sender moves" << endl;
    Receiver.Move(StepSize);
	//cout << "Receiver moves" << endl;
    Sender.Update(Receiver);
	//cout << "Sender updates" << endl;
    Receiver.Update(Sender);
	//cout << "Receiver updates" << endl;
#if !TARGET_ADDRESS_ONLY
    Sender.UpdateTargetVector(targetPosition);
#endif
	//cout << "Sender updates target" << endl;
	Receiver.UpdateHomeVector();
	//cout << "Receiver updates home" << endl;
}

void StepReceiverOnly(CommAgent1DLine &Receiver) {
    Receiver.Move(StepSize);
	Receiver.UpdateHomeVector();
}

void StepSenderOnly(CommAgent1DLine &Sender) {
#if CONSTRAIN_SENDER
	Sender.MoveConstrained(StepSize, currentLeftLimit, currentRightLimit);
#else
	Sender.Move(StepSize);
#endif
}

// returns a normalized position between [0, lineLength)
double normalizePosition(double position) {
	return position - floor(position/(WorldLength))*WorldLength;
}

// returns the separation between two positions. Since pos are normalized, sep also is.
double linearSeparation(double position1, double position2) {
	//cout << "In lineSep" << endl;
    double lsep = fabs(position1 - position2);
	double rsep = WorldLength - lsep;
	//cout << "linear separation calculated" << endl;
    return min(lsep, rsep);
}
#endif //All LINE methods --------------------------------------------------------------

#ifdef CIRCLE //All CIRCLE methods --------------------------------------------------------------
void PrepareCircleWorld()
{
#if CONSTRAIN_SENDER
	// set the clockwise and counterclockwise limits on the sender's motion
	CCWLimit = normalizeAngle(SenderRegion/2);
	CWLimit = normalizeAngle(-SenderRegion/2);
	if (CCWLimit == 0) CCWLimit = 2*Pi;
#endif
    
    // assign settings for the evaluation trials
	
    // initial positions for the sender 
    initSenderAngles.SetSize(NumInitSenderAngles);
#if CONSTRAIN_SENDER
    // if the sender is constrained, distribute the starting positions
    // within the constrained region
    for (int i = 1; i <= NumInitSenderAngles; i++)
		initSenderAngles[i] = CWLimit + i*SenderRegion/(NumInitSenderAngles+1);
#else
    // otherwise, distribute the positions uniformly around the arena
    for (int i = 1; i <= NumInitSenderAngles; i++)
		initSenderAngles[i] = normalizeAngle((i-1)*2*Pi/NumInitSenderAngles);
#endif
    
    // initial separations between the sender and receiver
    initAgentSeparations.SetSize(NumInitAgentSeparations);
    for (int i = 1; i <= NumInitAgentSeparations; i++)
        initAgentSeparations[i] = -MaxInitSeparation + (i-1)*2*MaxInitSeparation/(NumInitAgentSeparations-1);
	//SM: 2*MaxInitSeparation is to spread the separations on either sides of the agents
    
    // target locations
    targetAngles.SetSize(NumTargetAngles);
#if CONSTRAIN_SENDER
    // if the sender is constrained, distribute the targets on the opposite
    // side of the circular arena
    for (int i = 1; i <= NumTargetAngles; i++) //SM: target range is (90,270) deg coz init sender is near 0 deg
        targetAngles[i] = Pi/2 + (i-1)*Pi/(NumTargetAngles-1); 
#else
    // otherwise, distribute the targets uniformly around the arena
    for (int i = 1; i <= NumTargetAngles; i++)
        targetAngles[i] = (i-1)*2*Pi/NumTargetAngles;        
#endif
	
    NumTrials = initSenderAngles.Size() * initAgentSeparations.Size() * targetAngles.Size();
}

double RunAgents(CommAgent1DCircle &Sender, CommAgent1DCircle &Receiver)
{
    double Total = 0;
    for (int senderAngIndex = 1; senderAngIndex <= initSenderAngles.Size(); senderAngIndex++) {
        for (int agentSepIndex = 1; agentSepIndex <= initAgentSeparations.Size(); agentSepIndex++) {
            for (int targetAngIndex = 1; targetAngIndex <= targetAngles.Size(); targetAngIndex++) {
              Sender.SetAngularPosition(initSenderAngles[senderAngIndex]);
              Receiver.SetAngularPosition(initSenderAngles[senderAngIndex] + initAgentSeparations[agentSepIndex]);
              Total += RunTrial(Sender, Receiver, targetAngles[targetAngIndex]);
            }
        }
    }
    return Total / NumTrials;
}

double RunTrial(CommAgent1DCircle &Sender, CommAgent1DCircle &Receiver, double targetAngle)
{	
    double diff1, diff2;
    INITTRIALFUNCTION(Sender, Receiver);
#if ONE_INTERACTION_ONLY
        double time = 0, agentSep = angleSeparation(Sender.AngularPosition(),
                                                    Receiver.AngularPosition());
        while (time < TrialDuration && agentSep <= MaxSensorAngle) {
            StepBothAgents(Sender, Receiver, targetAngle); 
            agentSep = angleSeparation(Sender.AngularPosition(), Receiver.AngularPosition());
            time += StepSize;
        }
        Receiver.Update(Sender);
        while (time < TrialDuration) {
            StepReceiverOnly(Receiver);
            time += StepSize;
        }
        diff1 = angleSeparation(Receiver.AngularPosition(), targetAngle);  
        time = 0, agentSep = angleSeparation(Sender.AngularPosition(),
                                             Receiver.AngularPosition());
        while (time < PostTrialInterval && agentSep <= MaxSensorAngle) {
            StepBothAgents(Sender, Receiver, targetAngle);
            agentSep = angleSeparation(Sender.AngularPosition(), Receiver.AngularPosition());
            time += StepSize;
        }
        Receiver.Update(Sender);
        while (time < PostTrialInterval) {
            StepReceiverOnly(Receiver);
            time += StepSize;
        }
        diff2 = angleSeparation(Receiver.AngularPosition(), targetAngle);
#else
        for (double time = 0; time < TrialDuration; time += StepSize)
            StepBothAgents(Sender, Receiver, targetAngle);
        diff1 = angleSeparation(Receiver.AngularPosition(), targetAngle);
        for (double time = 0; time < PostTrialInterval; time += StepSize)
            StepBothAgents(Sender, Receiver, targetAngle);
        diff2 = angleSeparation(Receiver.AngularPosition(), targetAngle);
#endif
    return max(1.0 - (diff1 + diff2) / (2*Pi), 0.0);
}

void StepBothAgents(CommAgent1DCircle &Sender, CommAgent1DCircle &Receiver, double targetAngle) {
#if CONSTRAIN_SENDER
        Sender.MoveConstrained(StepSize, CCWLimit, CWLimit);
#else
        Sender.Move(StepSize);
#endif
    Receiver.Move(StepSize);
    Sender.Update(Receiver);
    Receiver.Update(Sender);
    Sender.UpdateTargetVector(targetAngle);
	Receiver.UpdateHomeVector();
}

void StepReceiverOnly(CommAgent1DCircle &Receiver) {
    Receiver.Move(StepSize);
	Receiver.UpdateHomeVector();
}

// returns a normalized angle between [0, 2*Pi) -- SM: same as angle % (2*Pi)
double normalizeAngle(double angle) {
    return angle - floor(angle/(2*Pi))*2*Pi;
}

// returns the separation between two angles, normalized to [0, Pi] -- SM: same as angle % (2*Pi)
double angleSeparation(double angle1, double angle2) {
    double result = normalizeAngle(angle1 - angle2);
    if (result > Pi) result = 2*Pi - result; //SM: This is needed because there are 2 sensors and each can span only a max of pi
    return result;
}
#endif //All CIRCLE methods --------------------------------------------------------------

void LoadCircuitParameters(SensorCTRNN &SenderNervousSystem,
                           SensorCTRNN &ReceiverNervousSystem, TVector<double> &v)
{
    int const FirstRay = 1, LastRay = 2; //SM: Agents' angular sensors
#if SINGLE_BEARING_SENSOR
	int const FirstHVSensor = 3, LastHVSensor = 3; //SM: Receiver's bearing sensors;HV=home vector
    int const FirstTVSensor = 3, LastTVSensor = 3; //SM: Sender's bearing sensors;TV=target vector
#else
    int const FirstHVSensor = 3, LastHVSensor = 4; //SM: Receiver's bearing sensors;HV=home vector
    int const FirstTVSensor = 3, LastTVSensor = 4; //SM: Sender's bearing sensors;TV=target vector
#endif
    int const FirstInt = 1, LastInt = NUMINTS;
	//In the LINE world, CounterClockwiseMotor = LeftMotor and ClockwiseMotor = RightMotor
    int const CounterClockwiseMotor = CIRCUITSIZE-1, ClockwiseMotor = CIRCUITSIZE;
    int VectorIndex = 1; //received as a reference inside the functions and updated there
    //double p;
    // Load the **SENDER**
    // Load the sensor weights
    // --- Ray sensors to interneurons
    LoadSensorWeights(SenderNervousSystem, FirstRay, LastRay, FirstInt, LastInt,
    v, VectorIndex, MinWeight, MaxWeight);
    // --- TV sensors to interneurons
#if PARTIAL_SENDER_BEARING  //project bearing only to the 3rd interneuron
	LoadSensorWeights(SenderNervousSystem, FirstTVSensor, LastTVSensor, 3,
					  3, v, VectorIndex, MinWeight, MaxWeight);
#else
    LoadSensorWeights(SenderNervousSystem, FirstTVSensor, LastTVSensor, FirstInt,
    LastInt, v, VectorIndex, MinWeight, MaxWeight);
#endif
    // Load the circuit weights
    // --- Recurrent connections within interneurons
    LoadCircuitWeights(SenderNervousSystem, FirstInt, LastInt, FirstInt, LastInt,
    v, VectorIndex, MinWeight, MaxWeight);
    // --- Interneurons to motor neurons
    LoadCircuitWeights(SenderNervousSystem, FirstInt, LastInt, CounterClockwiseMotor,
    ClockwiseMotor, v, VectorIndex, MinWeight, MaxWeight);	
    // Load the circuit biases
    LoadBiases(SenderNervousSystem, FirstInt, LastInt, v, VectorIndex, MinBias,
    MaxBias);
    LoadBiases(SenderNervousSystem, CounterClockwiseMotor, ClockwiseMotor, v,
    VectorIndex, MinBias, MaxBias);
    // Load the time constants
    LoadTimeConstantsExp(SenderNervousSystem, FirstInt, LastInt, v, VectorIndex,
    MinTau, MaxTau);
    LoadTimeConstantsExp(SenderNervousSystem, CounterClockwiseMotor, ClockwiseMotor, v,
    VectorIndex, MinTau, MaxTau);
    // Load the **RECEIVER**
#if !DISTINCT_AGENTS //the same ctrnn is installed in both the sender and the receiver
	VectorIndex = 1;
#endif
	//cout << VectorIndex << endl;
    // Load the sensor weights
    // --- Ray sensors to interneurons
    LoadSensorWeights(ReceiverNervousSystem, FirstRay, LastRay, FirstInt, LastInt,
    v, VectorIndex, MinWeight, MaxWeight);
    // --- HV sensors to interneurons
    LoadSensorWeights(ReceiverNervousSystem, FirstHVSensor, LastHVSensor, FirstInt,
    LastInt, v, VectorIndex, MinWeight, MaxWeight);
    // Load the circuit weights
    // --- Recurrent connections within interneurons
    LoadCircuitWeights(ReceiverNervousSystem, FirstInt, LastInt, FirstInt, LastInt,
    v, VectorIndex, MinWeight, MaxWeight);
    // --- Interneurons to motor neurons
    LoadCircuitWeights(ReceiverNervousSystem, FirstInt, LastInt, CounterClockwiseMotor,
    ClockwiseMotor, v, VectorIndex, MinWeight, MaxWeight);	
    // Load the circuit biases
    LoadBiases(ReceiverNervousSystem, FirstInt, LastInt, v, VectorIndex, MinBias,
    MaxBias);
    LoadBiases(ReceiverNervousSystem, CounterClockwiseMotor, ClockwiseMotor, v,
    VectorIndex, MinBias, MaxBias);
    // Load the time constants
    LoadTimeConstantsExp(ReceiverNervousSystem, FirstInt, LastInt, v, VectorIndex,
    MinTau, MaxTau);
    LoadTimeConstantsExp(ReceiverNervousSystem, CounterClockwiseMotor, ClockwiseMotor, v,
    VectorIndex, MinTau, MaxTau);
}
	
// Dump the best circuits to a file
void DumpBestCircuit(int generation, TVector<double> &v)
{
#ifdef CIRCLE
	AGENT Sender(MaxAngularVelocity, MaxSensorAngle, CIRCUITSIZE, SENSORSIZE);
    AGENT Receiver(MaxAngularVelocity, MaxSensorAngle, CIRCUITSIZE, SENSORSIZE);
#endif
#ifdef LINE
	AGENT Sender(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
    AGENT Receiver(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
#endif
    LoadCircuitParameters(Sender.NervousSystem, Receiver.NervousSystem, v);
    ofstream ofs(senderresultfname);
    ofs << Sender;
    ofs.close();
    ofs.open(receiverresultfname);
    ofs << Receiver;
    ofs.close();
    //cout << "Sender\n" << Sender << "\n";
    //cout << "Receiver\n" << Receiver << "\n";
}

#ifdef TRACK_LINEAGES
void DisplayLineage(TSearch &s, treeNode node, ofstream &ofs) {
    if (node != NULL) {
        DisplayLineage(s, s.Parent(node), ofs);
        ofs << (*node).generation << " " << EvaluateAgents((*node).individual) << endl;
        DumpFinalCircuits((*node).generation, (*node).individual);
    }
}

void DumpFinalCircuits(int generation, TVector<double> &v) {    
    CommAgent1DCircle Sender(MaxAngularVelocity, MaxSensorAngle, CIRCUITSIZE, SENSORSIZE);
    CommAgent1DCircle Receiver(MaxAngularVelocity, MaxSensorAngle, CIRCUITSIZE, SENSORSIZE);
    LoadCircuitParameters(Sender.NervousSystem, Receiver.NervousSystem, v);
    char senderfname[50], receiverfname[50];
    sprintf(senderfname, "%s/bestSender%d.ns", lineagefolder.c_str(), generation);
    sprintf(receiverfname, "%s/bestReceiver%d.ns", lineagefolder.c_str(), generation);
    ofstream ofs(senderfname);
    ofs << Sender;
    ofs.close();
    ofs.open(receiverfname);
    ofs << Receiver;
    ofs.close();  
}
#endif

