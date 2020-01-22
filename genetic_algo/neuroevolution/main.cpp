// ***************************************
// Evolving a CTRNN to produce activation patterns
//
// Developed by Saber Sheybani - Spring 2017
// ***************************************

#include "CTRNN.h"
#include <math.h>
#include "TSearch.h"
#include <iostream>///
#include <iomanip>
#include <time.h>
#include "VectorList.h"

/// Global Variables
/// Set number of neurons
const int nneuron = 3;
/// For Running Neural Network
const double RunDuration = 150;
const double StepSize = 0.01;


void RunNeuralNet(TVector<double> &v){

  // Parse and Map
  vector<double> time_constants(nneuron);
  vector<double> bias_points(nneuron);
  vector<double> connection_weights(pow(nneuron,2));

  int index = 0;
  for (int it = 0; it<nneuron; it++){
    time_constants[index] = MapSearchParameter(v[it],0.1,10);
    index++;
  }

  index = 0;
  for (int it = nneuron; it<2*nneuron; it++){
    bias_points[index] = MapSearchParameter(v[it],-15,15);
    index++;
  }

  index = 0;
  for (int it = 2*nneuron; it<v.Size(); it++){
    connection_weights[index] = MapSearchParameter(v[it],-15,15);
    index++;
  }

  /// NN
  CTRNN c(nneuron);

  /// Set the parameters of neurons.
    for (int i = 1; i <= nneuron; i++){
      c.SetNeuronTimeConstant(i, time_constants[i-1]);
      c.SetNeuronBias(i, bias_points[i-1]);
        for (int j = 1; j <= nneuron; j++)
            c.SetConnectionWeight(i, j, connection_weights[nneuron*(i-1)+j-1]);
    }
    // Run the circuit
    c.RandomizeCircuitState(-0.5, 0.5);

    ofstream NNRunFile;
    NNRunFile.open("nn_run_states.dat",ios_base::trunc);
    NNRunFile << setprecision(6);


    for (double time = StepSize; time <= RunDuration; time += StepSize) {
        c.EulerStep(StepSize);
        NNRunFile << c.states << endl;
    }
    NNRunFile.close();
}


// ------------------------------------
// Display functions
// ------------------------------------
void EvolutionaryRunDisplay(int Generation, double BestPerf, double AvgPerf, double PerfVar)
{
  cout << "Generation: " << Generation << " Best Performance = " << BestPerf << endl;
  cerr << "Generation: " << Generation << " Best Performance = " << BestPerf << endl;
}

  // Write the best individual in a file
  void ResultsDisplay(TSearch &s)
{
    TVector<double> bestVector;
    ofstream BestIndividualFile;

    // Get the parameter Vector (n x Tau, n x Bias, Weights)
    bestVector = s.BestIndividual();

    BestIndividualFile.open("best.gen.dat",ios_base::app);
    BestIndividualFile << setprecision(32);
    BestIndividualFile << bestVector << endl;
    BestIndividualFile << endl; // In order to separate multiple runs of the program.
    BestIndividualFile.close();

    /// Run the Neural Network again for the best parameter vector and this time, save the data.
    RunNeuralNet(bestVector);
}


double OscillationScore(const list<double> sig){
    if (sig.size() != 3)
        cerr << "The list should be of size 3" << endl;
    list<double> sig_copy = sig;
    sig_copy.pop_front(); // Only to have access to the middle element of the list,
                          // which is not available without using an iterator.
    double score_h = (sig.front() - sig_copy.front()) * (sig_copy.front() - sig_copy.back());
    if (score_h < 0)
        return -100*score_h;
    else
        return 0;
}


double NeuralNetScore(vector<double> time_constants, vector<double> bias_points, vector<double> connection_weights, bool save_flag = false)
{
    CTRNN c(nneuron);

	/// Set the parameters of neurons.
    for (int i = 1; i <= nneuron; i++){
    	c.SetNeuronTimeConstant(i, time_constants[i-1]);
    	c.SetNeuronBias(i, bias_points[i-1]);
        for (int j = 1; j <= nneuron; j++)
            c.SetConnectionWeight(i, j, connection_weights[nneuron*(i-1)+j-1]);
    }

    // Run the circuit
    c.RandomizeCircuitState(-0.5, 0.5);

    VecList states_sig = VecList(nneuron);
    vector<double> score_vec(nneuron, 0);
    //cout << 0.0 << " " << c.NeuronOutput(1) << " " << c.NeuronOutput(2) << endl;
    for (double time = StepSize; time <= RunDuration; time += StepSize) {
        c.EulerStep(StepSize);
/*
        /// Uncomment if the whole signal needs to be saved to file.
        if (save_flag == true)
          cout << c.states;
*/
        /// Accumulate signal data and Calculate Score.
        states_sig.push_back(c.states);
    	int i = 0;
    	for (vector<list<double> >::iterator vecit = states_sig.data_array.begin(); vecit != states_sig.data_array.end(); ++vecit){
        	score_vec[i] += OscillationScore(*vecit);
        	i++;
    	}
    }

    // Finished
    double final_score = 0;
    for (int i = 0; i < score_vec.size(); i++)
    	final_score += score_vec[i];

    return final_score;
}


double Evaluate(TVector<double> &v, RandomState &)
{
	vector<double> time_constants(nneuron);
	vector<double> bias_points(nneuron);
	vector<double> connection_weights(pow(nneuron,2));

	int index = 0;
	for (int it = 0; it<nneuron; it++){
		time_constants[index] = MapSearchParameter(v[it],0.1,10);
		index++;
	}

	index = 0;
	for (int it = nneuron; it<2*nneuron; it++){
		bias_points[index] = MapSearchParameter(v[it],-15,15);
		index++;
	}

	index = 0;
	for (int it = 2*nneuron; it<v.Size(); it++){
		connection_weights[index] = MapSearchParameter(v[it],-15,15);
		index++;
	}


	return NeuralNetScore(time_constants, bias_points, connection_weights);
}


// The main program

////#########################
int main(int argc, const char* argv[]){

    int nparam = pow(nneuron,2) + 2*nneuron;
    cout << "Number of Neurons = " << nneuron << endl;
    cout << "Number of Parameters = " << nparam <<endl;

    TSearch s(nparam);
    std::cout << "Initialized to " << s.BestIndividual() << endl;
    long randomseed = static_cast<long>(time(NULL));
    // save the seed to a file
    ofstream seedfile;
    seedfile.open ("seed.dat",ios_base::app);
    seedfile << randomseed << endl;
    seedfile.close();
  // Configure the search
    s.SetRandomSeed(randomseed);
    s.SetEvaluationFunction(Evaluate); /// #
    s.SetSelectionMode(RANK_BASED);///
    s.SetReproductionMode(HILL_CLIMBING); /// #
    s.SetPopulationSize(200); ///
    s.SetMaxGenerations(20); /// Number of Generations
    s.SetMutationVariance(0.2); ///#
    s.SetCrossoverProbability(0.5); /// #
    s.SetCrossoverMode(TWO_POINT); /// #
    s.SetMaxExpectedOffspring(1.1); /// #
    s.SetElitistFraction(0.1); /// #
    s.SetSearchConstraint(1); ///
    s.SetCheckpointInterval(5); /// #

    s.SetPopulationStatisticsDisplayFunction(EvolutionaryRunDisplay);
    s.SetSearchResultsDisplayFunction(ResultsDisplay);

    ofstream evolfile;
    evolfile.open ("fitness_generation.dat");
    cout.rdbuf(evolfile.rdbuf());

    // Run the search
    s.ExecuteSearch(); /// #
    // Display the best individual found
    cerr << "Best Individual = " << s.BestIndividual() << endl;
    cerr << "Best Performance = " << s.BestPerformance() << endl;
    return 0;
}

