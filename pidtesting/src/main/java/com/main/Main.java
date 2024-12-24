package com.main;

import java.util.function.Function;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class Main {

    // Numerical approximation of Laplace transform (untested)
    public static class LaplaceTransform {
        public static double laplaceTransform(Function<Double, Double> f, double s) {
            double result = 0;
            double dt = 0.001; // Step size for numerical integration
            for (double t = 0; t < 100; t += dt) { // Integrate from 0 to a large value
                result += f.apply(t) * Math.exp(-s * t) * dt;
            }
            return result;
        }
    }

    private static double feedForwardFunction(double y) {
        return Math.cos(y);
    }

    private static double[] ODELinearized(double[] input, double target, double P, double I, double D, double m) {
        // y = position, a = accumulated error, b = velocity
        double y = input[0];
        double a = input[1];
        double b = input[2];
        double dy = b;
        double da = y;
        double db = (-D * b - I * a - P * (y-target) - feedForwardFunction(y)) / m;
        return new double[]{dy, da, db};
    }

    public static double[] rk4Onestep(double[] currentState, double target, double P, double I, double D, double m, double dt) {
        double[] k1 = ODELinearized(currentState, target, P, I, D, m);
        double[] k2 = ODELinearized(add(currentState, mult(k1, dt / 2)), target, P, I, D, m);
        double[] k3 = ODELinearized(add(currentState, mult(k2, dt / 2)), target, P, I, D, m);
        double[] k4 = ODELinearized(add(currentState, mult(k3, dt)), target, P, I, D, m);

        return add(currentState, mult(add(k1, mult(k2, 2), mult(k3, 2), k4), dt / 6));
    }

    public static double[] add(double[]... arrays) {
        int length = arrays[0].length;
        double[] result = new double[length];
        for (double[] array : arrays) {
            for (int i = 0; i < length; i++) {
                result[i] += array[i];
            }
        }
        return result;
    }

    public static double[] mult(double[] arr, double scalar) {
        double[] result = new double[arr.length];
        for (int i = 0; i < arr.length; i++) {
            result[i] = arr[i] * scalar;
        }
        return result;
    }

    public static void main(String[] args) {
        // Simulation settings
        double finalTime = 10; // seconds
        double dt = 0.001; // time step
        int length = (int) Math.ceil(finalTime / dt);
        double[] initialConditions = {1, 0, 1}; // Initial y, a, b

        double[][] results = new double[length][];
        double target = -4.0;
        results[0] = initialConditions;

        // PID constants    // UNITS - must be consistent across program
        double P = 1000;     // force/position
        double I = 0;       // force/accumulated error
        double D = 300;      // force/velocity
        double m = 10;      // mass

        // Run the simulation
        for (int i = 1; i < length; i++) {
            results[i] = rk4Onestep(results[i - 1], target, P, I, D, m, dt);
        }

        // Create and display the chart
        XYSeries ySeries = new XYSeries("Position (y)");
        //XYSeries aSeries = new XYSeries("Accumulated Error (a)");
        //XYSeries bSeries = new XYSeries("Velocity (b)");

        for (int i = 0; i < length; i++) {
            double time = i * dt;
            ySeries.add(time, results[i][0]);
            // aSeries.add(time, results[i][1]);
            // bSeries.add(time, results[i][2]);
        }

        XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(ySeries);
        // dataset.addSeries(aSeries);
        // dataset.addSeries(bSeries);

        JFreeChart chart = ChartFactory.createXYLineChart(
                "System Simulation Results",
                "Time (s)",
                "Values",
                dataset
        );

        JFrame frame = new JFrame("Simulation Results");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new ChartPanel(chart));
        frame.pack();
        frame.setVisible(true);
    }
}
