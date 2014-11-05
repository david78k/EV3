using System;

namespace ev3
{
	public class NeuralNetwork
	{
//		int nInputs = 2;
//		int nOutputs = 2;
		double[][] trainInputs = new double[][]{
			new double[]{0, 1}
		};
		double[][] trainOutputs = new double[][]{
			new double[]{1, 1}
		};
		double[][] weights; // need matrix?
		int epochs = 5;

		public NeuralNetwork ()
		{
			init();
		}

		public void learn() {
			Console.WriteLine("Training network for " + epochs + " epochs ...");

			int tInputs = trainInputs.Length;
			int tOutputs = trainOutputs.Length;
			Console.WriteLine("(tInputs, tOutputs) = (" + tInputs + ", " + tOutputs + ")");
			if (tInputs != tOutputs) {
				Console.WriteLine ("train data have inconsistent dimensions");
				return;
			}
			for (int epoch = 0; epoch < epochs; epoch ++) {
				Console.WriteLine("epoch " + epoch);
				for (int i = 0; i < tInputs; i++) {
					Console.WriteLine (i + " " + trainInputs[i]);
					learn (trainInputs[i], trainOutputs[i]);
					print (weights);
				}
			}
		}

		public void learn(double[] inputs, double[] targets) {
			double error = 0;
			double gradient = 0;
			double[] outputs = new double[targets.Length];

			// forward propagate
			for (int i = 0; i < inputs.Length; i++) {
				for (int j = 0; j < targets.Length; j++) {
					outputs [j] = inputs [i] * weights [i] [j];
					error = targets [j] - outputs [j];
					gradient = error * (1 - error);
				}
			}
			// compute errors

			// compute gradients

			// backward propagate
			for (int i = 0; i < inputs.Length; i++) {
				for (int j = 0; j < targets.Length; j++) {
					weights [i][j] += gradient;
				}
			}
		}

		public void test(double[] inputs, double[] targets) {
			int corrects = 0;
			double[] outputs = new double[targets.Length];
			for (int i = 0; i < inputs.Length; i ++) {
				for (int j = 0; j < outputs.Length; j++) {
					outputs [j] = inputs [i] * weights [i] [j];
					if (outputs [j] == targets [j])
						corrects++;
					else
						Console.WriteLine ("incorrect: " + outputs [j] + " - expected: " + targets [j]);
				}
			}
			Console.WriteLine ("corrects: " + corrects + "/" + targets.Length);
		}

		// initialize random weights
		public void init() {
			Console.WriteLine("trainInputs: ");
			print(trainInputs);
			Console.WriteLine("trainOutputs: ");
			print (trainOutputs);

			Console.WriteLine ();

			int nInputs = trainInputs[0].Length;
			int nOutputs = trainOutputs[0].Length;

			Console.WriteLine("Initializing weight matrix " + nInputs + " x " + nOutputs + " ...");
			weights = new double[nInputs][];
			Random rand = new Random(1);

			for (int i = 0; i < weights.Length; i ++) {
				weights[i] = new double[nOutputs];
				for (int j = 0; j < weights [i].Length; j++) {
					weights [i] [j] = rand.NextDouble ();
				}
			}
			print (weights);
		}

		public void print(double[][] matrix) {
			for (int i = 0; i < matrix.Length; i ++) {
				for (int j = 0; j < matrix [i].Length; j++) {
					Console.Write (matrix [i][j] + " ");
				}
				Console.WriteLine ();
			}
		}
	}
}

