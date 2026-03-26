// *****************************************************
// Utility functions for mapping from a search vector to
// SensorCTRNN parameters.
// *****************************************************

#include "SensorCTRNN.h"
#include "VectorMatrix.h"
#include "Search.h"

void LoadSensorWeightsWithBilateralSymmetry(SensorCTRNN &NervousSystem,
int FirstFrom, int LastFrom, int FirstTo, int LastTo, TVector<double> &v,
int &VectorIndex, double MinW, double MaxW) {
    double weight;
    int i, j;
    int midLine = (LastFrom - FirstFrom) / 2;
    int numTo = LastTo - FirstTo + 1;
    for(i = 0; i < midLine; i++) {
        for(j = 0; j < numTo; j++) {
            weight = LinearMapSearchParameter(v[VectorIndex++], MinW, MaxW);
            NervousSystem.SetSensorWeight(FirstFrom + i, FirstTo + j, weight);
            NervousSystem.SetSensorWeight(LastFrom - i, LastTo - j, weight);
        }
    }
    numTo = (numTo + 1) / 2;
    for(int j = 0; j < numTo; j++) {
        weight = LinearMapSearchParameter(v[VectorIndex++], MinW, MaxW);
        NervousSystem.SetSensorWeight(FirstFrom + i, FirstTo + j, weight);
        NervousSystem.SetSensorWeight(LastFrom - i, LastTo - j, weight);
    }
}

void LoadCircuitWeightsWithBilateralSymmetry(SensorCTRNN &NervousSystem,
int FirstFrom, int LastFrom, int FirstTo, int LastTo, TVector<double> &v,
int &VectorIndex, double MinW, double MaxW) {
    double weight;
    int i, j;
    int midLine = (LastFrom - FirstFrom) / 2;
    int numTo = LastTo - FirstTo + 1;
    for(i = 0; i < midLine; i++) {
        for(j = 0; j < numTo; j++) {
            weight = LinearMapSearchParameter(v[VectorIndex++], MinW, MaxW);
            NervousSystem.SetCircuitWeight(FirstFrom + i, FirstTo + j, weight);
            NervousSystem.SetCircuitWeight(LastFrom - i, LastTo - j, weight);
        }
    }
    numTo = (numTo + 1) / 2;
    for(int j = 0; j < numTo; j++) {
        weight = LinearMapSearchParameter(v[VectorIndex++], MinW, MaxW);
        NervousSystem.SetCircuitWeight(FirstFrom + i, FirstTo + j, weight);
        NervousSystem.SetCircuitWeight(LastFrom - i, LastTo - j, weight);
    }
}

void LoadBiasesWithBilateralSymmetry(SensorCTRNN &NervousSystem, int FirstUnit,
int LastUnit, TVector<double> &v, int &VectorIndex, double MinB,
double MaxB) {
    double bias;
    int midLine = (LastUnit - FirstUnit) / 2;
    for(int i = 0; i <= midLine; i++) {
        bias = LinearMapSearchParameter(v[VectorIndex++], MinB, MaxB);
        NervousSystem.SetNeuronBias(FirstUnit + i, bias);
        NervousSystem.SetNeuronBias(LastUnit - i, bias);
    }
}

void LoadTimeConstantsWithBilateralSymmetry(SensorCTRNN &NervousSystem, int FirstUnit,
int LastUnit, TVector<double> &v, int &VectorIndex, double MinT, double MaxT) {
    double tau;
    int midLine = (LastUnit - FirstUnit) / 2;
    for(int i = 0; i <= midLine; i++) {
        tau = LinearMapSearchParameter(v[VectorIndex++], MinT, MaxT, 1);
        NervousSystem.SetNeuronTimeConstant(FirstUnit + i, tau);
        NervousSystem.SetNeuronTimeConstant(LastUnit - i, tau);
    }
}

void LoadTimeConstantsExpWithBilateralSymmetry(SensorCTRNN &NervousSystem, int FirstUnit,
int LastUnit, TVector<double> &v, int &VectorIndex, double MinT, double MaxT) {
    double MinExp = log(MinT), MaxExp = log(MaxT);
    double tau;
    int midLine = (LastUnit - FirstUnit) / 2;
    for(int i = 0; i <= midLine; i++) {
        tau = ExpMapSearchParameter(v[VectorIndex++], MinExp, MaxExp);
        NervousSystem.SetNeuronTimeConstant(FirstUnit + i, tau);
        NervousSystem.SetNeuronTimeConstant(LastUnit - i, tau);
    }
}

void LoadSensorWeights(SensorCTRNN &NervousSystem, int FirstFrom, int LastFrom,
int FirstTo, int LastTo, TVector<double> &v, int &VectorIndex, double MinW,
double MaxW) {
    double weight;
    int i, j;
    int numFrom = LastFrom - FirstFrom + 1;
    int numTo = LastTo - FirstTo + 1;
    for(i = 0; i < numFrom; i++) {
        for(j = 0; j < numTo; j++) {
            weight = LinearMapSearchParameter(v[VectorIndex++], MinW, MaxW);
            NervousSystem.SetSensorWeight(FirstFrom + i, FirstTo + j, weight);
        }
    }
}

void LoadCircuitWeights(SensorCTRNN &NervousSystem, int FirstFrom, int LastFrom,
int FirstTo, int LastTo, TVector<double> &v, int &VectorIndex, double MinW,
double MaxW) {
    double weight;
    int i, j;
    int numFrom = LastFrom - FirstFrom + 1;
    int numTo = LastTo - FirstTo + 1;
    for(i = 0; i < numFrom; i++) {
        for(j = 0; j < numTo; j++) {
            weight = LinearMapSearchParameter(v[VectorIndex++], MinW, MaxW);
            NervousSystem.SetCircuitWeight(FirstFrom + i, FirstTo + j, weight);
        }
    }
}

void LoadBiases(SensorCTRNN &NervousSystem, int FirstUnit, int LastUnit,
TVector<double> &v, int &VectorIndex, double MinB, double MaxB) {
    double bias;
    int numUnits = LastUnit - FirstUnit + 1;
    for(int i = 0; i < numUnits; i++) {
        bias = LinearMapSearchParameter(v[VectorIndex++], MinB, MaxB);
        NervousSystem.SetNeuronBias(FirstUnit + i, bias);
    }
}

void LoadTimeConstants(SensorCTRNN &NervousSystem, int FirstUnit, int LastUnit,
TVector<double> &v, int &VectorIndex, double MinT, double MaxT) {
    double tau;
    int numUnits = LastUnit - FirstUnit + 1;
    for(int i = 0; i < numUnits; i++) {
        tau = LinearMapSearchParameter(v[VectorIndex++], MinT, MaxT, 1);
        NervousSystem.SetNeuronTimeConstant(FirstUnit + i, tau);
    }
}

void LoadTimeConstantsExp(SensorCTRNN &NervousSystem, int FirstUnit, int LastUnit,
TVector<double> &v, int &VectorIndex, double MinT, double MaxT) {
    double MinExp = log(MinT), MaxExp = log(MaxT);
    double tau;
    int numUnits = LastUnit - FirstUnit + 1;
    for(int i = 0; i < numUnits; i++) {
        tau = ExpMapSearchParameter(v[VectorIndex++], MinExp, MaxExp);
        NervousSystem.SetNeuronTimeConstant(FirstUnit + i, tau);
    }
}
