/*
 *  AnalysisMain.cpp
 *  CommunicationAnalysis
 *
 *  Created by Santosh Manicka on 11/22/11.
 *  Copyright 2011 Indiana University Bloomington. All rights reserved.
 *
 */
#include <cstdlib>
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "CommAgent1D.h"
#include "globals.h"

#define STEP_R_ONLY false

#define AGENT CommAgent1DLine
#define INITTRIALFUNCTION InitializeSearchTrial
using namespace std;

void InitializeSearchTrial(AGENT &Sender, AGENT &Receiver);
double RunAgents(AGENT &Sender, AGENT &Receiver);
double RunTrial(AGENT &Sender, AGENT &Receiver, double target);
void StepBothAgents(AGENT &Sender, AGENT &Receiver, double target);
void StepReceiverOnly(AGENT &Receiver);
void StepSenderOnly(AGENT &Sender);
void PrepareLineWorld();
double normalizePosition(double position);
double linearSeparation(double position1, double position2);
void OpenRecordingFile(int TrialNum);
void CloseRecoringFile();

//Globals
TVector <double> initSenderPositions, initAgentSeparations, targetPositions;
TVector <double> leftLimits, rightLimits;
double currentLeftLimit, currentRightLimit;
int NumTrials;
string runNum = "Run89";
string senderresultfname;
string receiverresultfname;
stringstream filename(stringstream::out|stringstream::in);
string RecordingFileName;
ofstream RecordingFile;
double TrialTimeStep = 0.0;

int main(int argc, char *argv[]) {  
	filename << "./" << runNum << "/bestSender.ns";
	senderresultfname = filename.str();
	cout << senderresultfname << endl;
	filename.str(""); //flush
	filename << "./" << runNum << "/bestReceiver.ns";
	receiverresultfname = filename.str();
	cout << receiverresultfname << endl;
	
	AGENT Sender(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
    AGENT Receiver(WorldLength, MaxLinearVelocity, MaxSensorDist, CIRCUITSIZE, SENSORSIZE);
	ifstream senderfile(senderresultfname.data());
	senderfile >> Sender;
	ifstream receiverfile(receiverresultfname.data());
	receiverfile >> Receiver;
	
	cout << Sender << endl;
	cout << Receiver << endl;
	
	/*
	filename.str("");
	filename << "./" << runNum << "/SRtrail.csv";
	SRtrailfilename = filename.str();
	SRtrailfile.open(SRtrailfilename.data(),ios::out);
	*/
	
	PrepareLineWorld();
	//cout << "Line prepared" << endl;
	cout << "Performance = " << RunAgents(Sender, Receiver) << endl;
	
}
	
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
	
	//SRtrailfile << NumTrials << "," << InfoTransmissionInterval << "," << TrialDuration << "," << PostTrialInterval << endl;
}

void InitializeSearchTrial(AGENT &Sender, AGENT &Receiver) {
    Sender.NervousSystem.Initialize();
    Receiver.NervousSystem.Initialize();
}

void OpenRecordingFile(int TrialNum) {
	stringstream Trial;
	Trial << TrialNum;
	filename.str("");
	filename << "./" << runNum << "/Recordings/T" << Trial.str() << ".csv";
	RecordingFileName = filename.str();
	RecordingFile.open(RecordingFileName.data(),ios::out);
}

void CloseRecordingFile() {
	RecordingFile.close();
}

void RecordData(CommAgent1DLine Sender, CommAgent1DLine Receiver, double time) {
	RecordingFile << time << ",";
	for (int i = 1; i <= SENSORSIZE; i++)
		RecordingFile << Sender.NervousSystem.Sensor(i) << ",";
	for (int i = 1; i <= CIRCUITSIZE; i++)
		RecordingFile << Sender.NervousSystem.NeuronOutput(i) << ",";
	RecordingFile << Sender.linearPosition << "," << Sender.vLinear << ",";
	for (int i = 1; i <= SENSORSIZE; i++)
		RecordingFile << Receiver.NervousSystem.Sensor(i) << ",";
	for (int i = 1; i <= CIRCUITSIZE; i++)
		RecordingFile << Receiver.NervousSystem.NeuronOutput(i) << ",";
	RecordingFile << Receiver.linearPosition << "," << Receiver.vLinear << ",";
	RecordingFile << endl;
}
		
double RunAgents(CommAgent1DLine &Sender, CommAgent1DLine &Receiver) {
	//cout << "In RunAgents, Receiver's linear position = " << Receiver.linearPosition << endl;
    double Total = 0;
	int TrialNum = 1;
	for (int zone = 1; zone <= NumConstraintZones; zone++) {
		for (int targetPosIndex = 1; targetPosIndex <= NumTargetPositions; targetPosIndex++) {
			for (int senderPosIndex = 1; senderPosIndex <= NumInitSenderPositionsInZone; senderPosIndex++) {
				currentLeftLimit = leftLimits[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex];
				currentRightLimit = rightLimits[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex];
				for (int agentSepIndex = 1; agentSepIndex <= initAgentSeparations.Size(); agentSepIndex++) {
					Sender.SetLinearPosition(initSenderPositions[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex]);
					//cout << "In RunAgents loop, Sender's linear position = " << Sender.linearPosition << endl;
					Receiver.SetLinearPosition(initSenderPositions[((zone-1)*NumInitSenderPositionsInZone)+senderPosIndex] + 
											   initAgentSeparations[agentSepIndex], NORMALIZE);
					//cout << "In RunAgents loop, Receiver's linear position = " << Receiver.linearPosition << endl;
					OpenRecordingFile(TrialNum++);
					RecordingFile << targetPositions[targetPosIndex] << endl;
					TrialTimeStep = 0.0;
					RecordData(Sender, Receiver, TrialTimeStep++);
					//SRtrailfile << Sender.linearPosition << "," << Receiver.linearPosition << "," << Receiver.vLinear << endl;
					Total += RunTrial(Sender, Receiver, targetPositions[targetPosIndex]);
					CloseRecordingFile();
					//cout << "Trial run once" << endl;
				}
			}
		}
	}
    return Total / NumTrials;
}

double RunTrial(CommAgent1DLine &Sender, CommAgent1DLine &Receiver, double targetPosition)
{	
	//cout << "Running trial" << endl;
	//cout << "In RunTrial, Receiver's linear position = " << Receiver.linearPosition << endl;
	double diff1, diff2;
	INITTRIALFUNCTION(Sender, Receiver);
#if CLAMP_RECEIVER
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
		RecordData(Sender, Receiver, TrialTimeStep++);
		//SRtrailfile << Sender.linearPosition << "," << Receiver.linearPosition << "," << Receiver.vLinear << endl;
	}
	//Sender disappers, so set receiver's angular sensors to zero
	Receiver.NervousSystem.SetSensor(1, 0.0);
	Receiver.NervousSystem.SetSensor(2, 0.0);
	//Receiver.linearPosition = 5.0;
	for (double time = 0; time < TrialDuration; time += StepSize)
	{
		StepReceiverOnly(Receiver);
		RecordData(Sender, Receiver, TrialTimeStep++);
		//SRtrailfile << Sender.linearPosition << "," << Receiver.linearPosition << "," << Receiver.vLinear << endl;
	}
	diff1 = linearSeparation(Receiver.LinearPosition(), targetPosition);
	for (double time = 0; time < PostTrialInterval; time += StepSize)
	{
		StepReceiverOnly(Receiver);
		RecordData(Sender, Receiver, TrialTimeStep++);
		//SRtrailfile << Sender.linearPosition << "," << Receiver.linearPosition << "," << Receiver.vLinear << endl;
	}
	diff2 = linearSeparation(Receiver.LinearPosition(), targetPosition);
#else
#if TARGET_ADDRESS_ONLY
	Sender.UpdateTargetVectorAddrOnly(targetPosition);
#endif
#if STEP_R_ONLY
	for (double time = 0; time < TrialDuration; time += StepSize)
	{
		StepReceiverOnly(Receiver);
		//SRtrailfile << Sender.linearPosition << "," << Receiver.linearPosition << "," << Receiver.vLinear << endl;
	}
	diff1 = linearSeparation(Receiver.LinearPosition(), targetPosition);
	for (double time = 0; time < PostTrialInterval; time += StepSize)
	{
		StepReceiverOnly(Receiver);
		//SRtrailfile << Sender.linearPosition << "," << Receiver.linearPosition << "," << Receiver.vLinear << endl;
	}
	diff2 = linearSeparation(Receiver.LinearPosition(), targetPosition);
#else
	for (double time = 0; time < TrialDuration; time += StepSize)
	{
		StepBothAgents(Sender, Receiver, targetPosition);
		//cout << "Stepped once" << endl;
	}
	diff1 = linearSeparation(Receiver.LinearPosition(), targetPosition);
	for (double time = 0; time < PostTrialInterval; time += StepSize)
		StepBothAgents(Sender, Receiver, targetPosition);
	diff2 = linearSeparation(Receiver.LinearPosition(), targetPosition);
#endif //STEP_R_ONLY
#endif //CLAMP_RECEIVER
	//cout << diff1 << diff2 << endl;
	//cout << diff1 << "," << diff2 << endl;
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
	//cout << "Sender = " << Sender.linearPosition << " Receiver = " << Receiver.linearPosition << endl;
	//SRtrailfile << Sender.linearPosition << "," << Receiver.linearPosition << "," << Receiver.vLinear << endl;
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
	//cout << position1 << "," << position2 << endl;
	//cout << "In lineSep" << endl;
	double sep1 = fabs(position1 - position2);
	double sep2 = WorldLength - sep1;
	//cout << "linear separation calculated" << endl;
	return min(sep1, sep2);
}
	
