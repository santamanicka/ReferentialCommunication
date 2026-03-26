// *************************************************************
// Global parameters that control the behavior of the simulation
//	
// PLW 06/03/09
// *************************************************************

//#define ANALYZE						//############IMPORANT#############

#ifndef ANALYZE
#include "Search.h"
#endif

#pragma once

#ifndef _PI_DEF
#define _PI_DEF
const double Pi = 3.141592654;
#endif

// World Parameters
#define LINE							//############IMPORANT#############

// Agent Parameters for CIRCLE world
#ifdef CIRCLE
#define AGENT CommAgent1DCircle
// Uncomment the following line for table-based fast sine and cosine w/ linear interpolation
#define FAST_TRIG
// the maximum angle at which one agent receives input from the other agent
const double MaxSensorAngle = Pi/8.;
// the maximum initial separation between the two agents
const double MaxInitSeparation = (3./4.)*MaxSensorAngle;
// the maximum angle that an agent can move through in a unit of time
const double MaxAngularVelocity = Pi/32.;
#endif

// Agent Parameters for LINE world
#ifdef LINE
#define AGENT CommAgent1DLine
const double WorldLength = 10.0;
const double MaxSensorDist = WorldLength/16.;
const double MaxInitSeparation = (3./4.)*MaxSensorDist;
const double MaxLinearVelocity = WorldLength/64.;
#endif

// Circuit Parameters
#define SINGLE_BEARING_SENSOR false		//############IMPORANT#############
//partial_bearing CAN'T be true without single_bearing being true
#define PARTIAL_SENDER_BEARING false	//############IMPORANT#############
#define NUMINTS 5 //SM: inter-neurons   //############IMPORANT#############
#define CIRCUITSIZE (NUMINTS+2) //SM: +2 motor neurons
#if SINGLE_BEARING_SENSOR
#define SENSORSIZE (2+1) //2 angle sensors and 1 bearing sensor 
#else
#define SENSORSIZE (2+2) //SM: 2 angle sensors and 2 bearing sensors
#endif

// Simulation controls
#define TARGET_ADDRESS_ONLY true			//############IMPORANT#############
#define CLAMP_RECEIVER true				//############IMPORANT#############
#define RUNAGENTFUNCTION RunAgents
#define INITTRIALFUNCTION InitializeSearchTrial
const double StepSize = 0.1;
#ifdef CIRCLE
const int NumTargetAngles = 10;
const int NumInitSenderAngles = 1;
// this should be an even number, so that the agents don't start out
// right on top of each other
const int NumInitAgentSeparations = 6;
#endif
#ifdef LINE								//############IMPORANT#############
const int NumConstraintZones = 2;
const int NumInitSenderPositionsInZone = 2;
// this should be an even number, so that the agents don't start out
// right on top of each other
const int NumInitAgentSeparations = 2;
const int NumTargetPositions = 8;
const double ZoneTargetRangeSeparation = 0.5;
#endif
#if CLAMP_RECEIVER
const double InfoTransmissionInterval = 60.0; //############IMPORANT#############
const double TrialDurationCoeff = 2.0;
#else
const double TrialDurationCoeff = 2.0;
#endif
// run time before receiver's angular separation from target is first measured;
// the receiver should be at the target by this point
#ifdef CIRCLE
const double TrialDuration =  2*2*Pi/MaxAngularVelocity; //SM: time taken to cover 4pi distance at max velocity
#endif
#ifdef LINE
const double TrialDuration = TrialDurationCoeff*WorldLength/MaxLinearVelocity;
#endif
// amount of time after TrialDuration before the receiver's angular separation
// from the target is again measured.  The separations at these two times are
// averaged to force the receiver to stay at the target.
const double PostTrialInterval = 15;	//############IMPORANT#############
// whether or not to impose a hard spatial constraint on the sender (sender
// can only move a restricted distance from its starting position)
#define CONSTRAIN_SENDER true			//############IMPORANT#############
// if the sender is constrained, the size of the constrained region to use
// (the sender can move a maximum of SenderRegion/2 in either direction away from
// its starting location)
#ifdef CIRCLE
const double SenderRegion = Pi/2;
#endif
#ifdef LINE
const double SenderRegion = WorldLength/4.;
#endif
// whether to limit the agents to one interaction, so that once they separate
// for the first time they are unable to sense each other again
#define ONE_INTERACTION_ONLY false

// Search parameters
#ifndef ANALYZE //then include all definitions below
#define GA_MODE	false					//############IMPORANT#############
// whether the same ctrnn is to be installed in both the sender and receiver
#define DISTINCT_AGENTS true			//############IMPORANT#############
const TSelectionMode SelectionMode = RANK_BASED;
#if GA_MODE
const TReproductionMode ReproductionMode = GENETIC_ALGORITHM;
const double ElitistFraction = 0.03;
#else
const TReproductionMode ReproductionMode = HILL_CLIMBING;
#endif
//SM:: # of params is 2*(something) because there are 2 agents coded in one genotype
#if DISTINCT_AGENTS
const int distinctAgents = 2;
#else
const int distinctAgents = 1;
#endif
#if PARTIAL_SENDER_BEARING  //2 angle sensors project to all INTS; bearing sensor projects to 3rd INT ONLY
const int NumSenderSensorWeights = (NUMINTS*(SENSORSIZE-1))+1; 
#else
const int NumSenderSensorWeights = NUMINTS*SENSORSIZE;
#endif
const int NumParametersSender = distinctAgents*(NumSenderSensorWeights // Sensor weights 
												+ NUMINTS*(NUMINTS+2) // Circuit weights
												+ NUMINTS+2 // Biases
												+ NUMINTS+2); // Time constants
const int NumParametersReceiver = distinctAgents*(NUMINTS*SENSORSIZE // Sensor weights 
												+ NUMINTS*(NUMINTS+2) // Circuit weights
												+ NUMINTS+2 // Biases
												+ NUMINTS+2); // Time constants
const int NumParameters = NumParametersSender + NumParametersReceiver;
#if DISTINCT_AGENTS
const int PopulationSize = 450;
#else
const int PopulationSize = 280;
#endif
const int MaxGenerations = 20000;
const double MutationVariance = 10.0*sqrt(NumParameters)/sqrt(69);
/*										//############IMPORANT#############
#if DISTINCT_AGENTS
const double MutationVariance = 7; //related to CIRCUITSIZE?
#else 
const double MutationVariance = 4.9;
#endif
*/
const double FitnessScalingMultiple = 1.003;
const int SearchFoldFlag = 1;
const int SearchConstrainedFlag = 0;
const int ReEvaluationFlag = 0;
extern long RandomSeed;
const double MinWeight = -16, MaxWeight = 16;
const double MinBias = -16, MaxBias = 16;
const double MinTau = 1, MaxTau = 30;
const int CheckPointInterval = 30;
#endif
