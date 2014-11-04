using System;

namespace ev3
{
	public class NeuralNetwork
	{
		double[] inputs;
		double[] outputs;
		double[] weights;
		int epochs = 5;

		public NeuralNetwork ()
		{
			init();
		}

		public void train() {
			train (epochs);
		}

		public void train(int epoch) {
			Console.WriteLine("training network for " + epoch + " epochs ...");
		}

		public void test(double[] inputs, double[] outputs) {

		}

		// initialize random weights
		public void init() {
			weights = new double[3*2];
			Random rand = new Random(1);

			for (int i = 0; i < weights.Length; i ++) {
				weights[i] = rand.NextDouble();
				Console.WriteLine (weights [i]);
			}
		}
	}
}

