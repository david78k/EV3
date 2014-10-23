package org.lejos.ev3.pcsample.charts;
import java.awt.Dimension;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.FileReader;
import java.rmi.RemoteException;
import java.util.Properties;

import javax.swing.JFrame;

import lejos.remote.ev3.RMISampleProvider;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class Graph extends JFrame {
	private static final long serialVersionUID = 1L;
	private static String title = "EV3 sampling";
	private XYSeries series;
	private XYSeriesCollection dataset = new XYSeriesCollection();
	private RemoteEV3 ev3;
	private RMISampleProvider sp;
	private String[] labels;
	private float frequency;
	
	public Graph(String host, String sensorClass, String portName, String category, String[] labels, 
			String units, float minValue, float maxValue, int windowWidth, int windowHeight, float frequency) throws Exception {
		super(title);
		ev3 = new RemoteEV3(host);
		this.labels = labels;
		this.frequency = frequency;
		sp = ev3.createSampleProvider(portName, sensorClass, null);
		series = new XYSeries(labels[0]);
		dataset.addSeries(series);
		JFreeChart chart = ChartFactory.createXYLineChart(title, category, units, (XYDataset) dataset, 
				PlotOrientation.VERTICAL, true, true, false);
		
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new Dimension(windowWidth, windowHeight));
		setContentPane(chartPanel);
		
		XYPlot plot = chart.getXYPlot();
		NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
		rangeAxis.setRange(minValue, maxValue);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    addWindowListener(new WindowAdapter() {
	    	 
	        @Override
	        public void windowClosing(WindowEvent e) {
	        	try {
					if (sp != null) sp.close();
					sp = null;
				} catch (RemoteException e1) {
					e1.printStackTrace();
				}
	        }
	    });
	}

	public void run() throws Exception {
		int x=0;
        while(sp != null) {
        	float[] sample = sp.fetchSample();
        	for(int i=0;i<labels.length;i++) {
        		series.add(x++,sample[i]);
        	}
    		Delay.msDelay(((int) (1000f/frequency)));
        }
	}
	
	public static void main(String[] args) throws Exception {
		Properties p = new Properties();
		p.load(new FileReader("graph.properties"));
		String[] labels = p.getProperty("labels").split(",");
		Graph demo = new Graph(p.getProperty("host"), p.getProperty("class"), p.getProperty("port"),  
				p.getProperty("category"), labels, p.getProperty("units"), Float.parseFloat(p.getProperty("min")), Float.parseFloat(p.getProperty("max")), 
                Integer.parseInt(p.getProperty("width")), Integer.parseInt(p.getProperty("height")), 
                Float.parseFloat(p.getProperty("frequency")));
        demo.pack();
        demo.setVisible(true);
        demo.run();
	}
}
