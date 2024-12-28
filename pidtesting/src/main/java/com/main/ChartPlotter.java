package com.main;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

/**
 * A class for creating various types of charts to visualize simulation results.
 * It uses the JFreeChart library to create line charts displaying data such as position, error, velocity, and acceleration over time.
 */
class ChartPlotter {

    /**
     * Creates a simple XY line chart for a single data series.
     * 
     * @param title The title of the chart.
     * @param xLabel The label for the x-axis (typically "Time").
     * @param yLabel The label for the y-axis (depends on the data).
     * @param results A 2D array containing simulation results where each row represents a time step.
     * @param startTime The start time of the simulation.
     * @param dt The time step between simulation points.
     * @param dataIndex The index of the data series to plot (e.g., 0 for position, 1 for error, etc.).
     */
    public void createChart(String title, String xLabel, String yLabel, double[][] results, double startTime, double dt, int dataIndex) {
        // Create a series for the data to be plotted
        XYSeries series = new XYSeries(title);
        for (int i = 0; i < results.length; i++) {
            double time = startTime + i * dt;
            series.add(time, results[i][dataIndex]);
        }

        // Add the series to a dataset
        XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(series);

        // Create the chart using JFreeChart
        JFreeChart chart = ChartFactory.createXYLineChart(
                title,  // chart title
                xLabel, // x-axis label
                yLabel, // y-axis label
                dataset // data
        );

        // Display the chart in a new window
        JFrame frame = new JFrame(title);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new ChartPanel(chart));
        frame.pack();
        frame.setVisible(true);
    }

    /**
     * Creates an XY line chart for acceleration data based on the differences in velocity over time.
     * 
     * @param title The title of the chart.
     * @param xLabel The label for the x-axis (typically "Time").
     * @param yLabel The label for the y-axis (typically "Acceleration").
     * @param results A 2D array containing simulation results where each row represents a time step.
     * @param startTime The start time of the simulation.
     * @param dt The time step between simulation points.
     */
    public void createAccelerationChart(String title, String xLabel, String yLabel, double[][] results, double startTime, double dt) {
        // Create a series for the acceleration data
        XYSeries series = new XYSeries(title);
        for (int i = 1; i < results.length; i++) {
            double time = startTime + i * dt;
            double acceleration = (results[i][2] - results[i - 1][2]) / dt; // Calculate acceleration
            series.add(time, acceleration);
        }

        // Add the series to a dataset
        XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(series);

        // Create the chart using JFreeChart
        JFreeChart chart = ChartFactory.createXYLineChart(
                title,  // chart title
                xLabel, // x-axis label
                yLabel, // y-axis label
                dataset // data
        );

        // Display the chart in a new window
        JFrame frame = new JFrame(title);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new ChartPanel(chart));
        frame.pack();
        frame.setVisible(true);
    }

    /**
     * Creates a combined XY line chart displaying multiple data series (position, error, velocity, and acceleration).
     * 
     * @param title The title of the chart.
     * @param xLabel The label for the x-axis (typically "Time").
     * @param results A 2D array containing simulation results where each row represents a time step.
     * @param startTime The start time of the simulation.
     * @param dt The time step between simulation points.
     */
    public void createCombinedChart(String title, String xLabel, double[][] results, double startTime, double dt) {
        // Create separate series for each data type
        XYSeries positionSeries = new XYSeries("Position (y)");
        XYSeries errorSeries = new XYSeries("Accumulated Error (a)");
        XYSeries velocitySeries = new XYSeries("Velocity (b)");
        XYSeries accelerationSeries = new XYSeries("Acceleration");

        // Populate the series with data
        for (int i = 0; i < results.length; i++) {
            double time = startTime + i * dt;
            positionSeries.add(time, results[i][0]);
            errorSeries.add(time, results[i][1]);
            velocitySeries.add(time, results[i][2]);
            if (i > 0) {
                double acceleration = (results[i][2] - results[i - 1][2]) / dt; // Calculate acceleration
                accelerationSeries.add(time, acceleration);
            }
        }

        // Create datasets for each series
        XYSeriesCollection positionDataset = new XYSeriesCollection(positionSeries);
        XYSeriesCollection errorDataset = new XYSeriesCollection(errorSeries);
        XYSeriesCollection velocityDataset = new XYSeriesCollection(velocitySeries);
        XYSeriesCollection accelerationDataset = new XYSeriesCollection(accelerationSeries);

        // Create axes for each data type
        NumberAxis positionAxis = new NumberAxis("Position (y)");
        positionAxis.setAutoRangeIncludesZero(false); 
        positionAxis.setRange(calculateAxisRange(positionSeries));

        NumberAxis errorAxis = new NumberAxis("Accumulated Error (a)");
        errorAxis.setAutoRangeIncludesZero(false);
        errorAxis.setRange(calculateAxisRange(errorSeries));

        NumberAxis velocityAxis = new NumberAxis("Velocity (b)");
        velocityAxis.setAutoRangeIncludesZero(false);
        velocityAxis.setRange(calculateAxisRange(velocitySeries));

        NumberAxis accelerationAxis = new NumberAxis("Acceleration");
        accelerationAxis.setAutoRangeIncludesZero(false);
        accelerationAxis.setRange(calculateAxisRange(accelerationSeries));

        // Set up renderers for each dataset
        XYLineAndShapeRenderer renderer1 = new XYLineAndShapeRenderer(true, false);
        XYLineAndShapeRenderer renderer2 = new XYLineAndShapeRenderer(true, false);
        XYLineAndShapeRenderer renderer3 = new XYLineAndShapeRenderer(true, false);
        XYLineAndShapeRenderer renderer4 = new XYLineAndShapeRenderer(true, false);

        // Create the plot and add datasets with corresponding renderers
        XYPlot plot = new XYPlot();
        plot.setDataset(0, positionDataset);
        plot.setRenderer(0, renderer1);
        plot.setRangeAxis(0, positionAxis);

        plot.setDataset(1, errorDataset);
        plot.setRenderer(1, renderer2);
        plot.setRangeAxis(1, errorAxis);

        plot.setDataset(2, velocityDataset);
        plot.setRenderer(2, renderer3);
        plot.setRangeAxis(2, velocityAxis);

        plot.setDataset(3, accelerationDataset);
        plot.setRenderer(3, renderer4);
        plot.setRangeAxis(3, accelerationAxis);

        // Shared time axis for all plots
        NumberAxis timeAxis = new NumberAxis(xLabel);
        plot.setDomainAxis(timeAxis);

        // Create the chart using JFreeChart
        JFreeChart combinedChart = new JFreeChart(title, JFreeChart.DEFAULT_TITLE_FONT, plot, true);

        // Display the chart in a new window
        JFrame frame = new JFrame(title);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new ChartPanel(combinedChart));
        frame.pack();
        frame.setVisible(true);
    }

    /**
     * Creates a combined XY line chart with optional data series.
     * Allows the user to select which data series to plot.
     * 
     * @param title The title of the chart.
     * @param xLabel The label for the x-axis (typically "Time").
     * @param results A 2D array containing simulation results where each row represents a time step.
     * @param startTime The start time of the simulation.
     * @param dt The time step between simulation points.
     * @param plotPosition Whether to include the position series in the chart.
     * @param plotError Whether to include the error series in the chart.
     * @param plotVelocity Whether to include the velocity series in the chart.
     * @param plotAcceleration Whether to include the acceleration series in the chart.
     */
    public void createCombinedChart(String title, String xLabel, double[][] results, double startTime, double dt,
                                    boolean plotPosition, boolean plotError, boolean plotVelocity, boolean plotAcceleration) {
        // Create separate series for each data type
        XYSeries positionSeries = new XYSeries("Position (y)");
        XYSeries errorSeries = new XYSeries("Accumulated Error (a)");
        XYSeries velocitySeries = new XYSeries("Velocity (b)");
        XYSeries accelerationSeries = new XYSeries("Acceleration");

        // Populate the series with data if selected by the flags
        for (int i = 0; i < results.length; i++) {
            double time = startTime + i * dt;
            if (plotPosition) positionSeries.add(time, results[i][0]);
            if (plotError) errorSeries.add(time, results[i][1]);
            if (plotVelocity) velocitySeries.add(time, results[i][2]);
            if (plotAcceleration && i > 0) {
                double acceleration = (results[i][2] - results[i - 1][2]) / dt; // Calculate acceleration
                accelerationSeries.add(time, acceleration);
            }
        }

        // Create datasets for the selected series
        XYSeriesCollection positionDataset = null;
        XYSeriesCollection errorDataset = null;
        XYSeriesCollection velocityDataset = null;
        XYSeriesCollection accelerationDataset = null;

        if (plotPosition) positionDataset = new XYSeriesCollection(positionSeries);
        if (plotError) errorDataset = new XYSeriesCollection(errorSeries);
        if (plotVelocity) velocityDataset = new XYSeriesCollection(velocitySeries);
        if (plotAcceleration) accelerationDataset = new XYSeriesCollection(accelerationSeries);

        // Create axes for the selected series
        NumberAxis positionAxis = null, errorAxis = null, velocityAxis = null, accelerationAxis = null;
        if (plotPosition) {
            positionAxis = new NumberAxis("Position (y)");
            positionAxis.setAutoRangeIncludesZero(false);
            positionAxis.setRange(calculateAxisRange(positionSeries));
        }
        if (plotError) {
            errorAxis = new NumberAxis("Accumulated Error (a)");
            errorAxis.setAutoRangeIncludesZero(false);
            errorAxis.setRange(calculateAxisRange(errorSeries));
        }
        if (plotVelocity) {
            velocityAxis = new NumberAxis("Velocity (b)");
            velocityAxis.setAutoRangeIncludesZero(false);
            velocityAxis.setRange(calculateAxisRange(velocitySeries));
        }
        if (plotAcceleration) {
            accelerationAxis = new NumberAxis("Acceleration");
            accelerationAxis.setAutoRangeIncludesZero(false);
            accelerationAxis.setRange(calculateAxisRange(accelerationSeries));
        }

        // Set up renderers
        XYLineAndShapeRenderer renderer1 = new XYLineAndShapeRenderer(true, false);
        XYLineAndShapeRenderer renderer2 = new XYLineAndShapeRenderer(true, false);
        XYLineAndShapeRenderer renderer3 = new XYLineAndShapeRenderer(true, false);
        XYLineAndShapeRenderer renderer4 = new XYLineAndShapeRenderer(true, false);

        // Create the plot and add datasets with corresponding renderers
        XYPlot plot = new XYPlot();
        int datasetIndex = 0;

        if (plotPosition) {
            plot.setDataset(datasetIndex, positionDataset);
            plot.setRenderer(datasetIndex, renderer1);
            plot.setRangeAxis(datasetIndex, positionAxis);
            datasetIndex++;
        }

        if (plotError) {
            plot.setDataset(datasetIndex, errorDataset);
            plot.setRenderer(datasetIndex, renderer2);
            plot.setRangeAxis(datasetIndex, errorAxis);
            datasetIndex++;
        }

        if (plotVelocity) {
            plot.setDataset(datasetIndex, velocityDataset);
            plot.setRenderer(datasetIndex, renderer3);
            plot.setRangeAxis(datasetIndex, velocityAxis);
            datasetIndex++;
        }

        if (plotAcceleration) {
            plot.setDataset(datasetIndex, accelerationDataset);
            plot.setRenderer(datasetIndex, renderer4);
            plot.setRangeAxis(datasetIndex, accelerationAxis);
        }

        // Shared time axis for all plots
        NumberAxis timeAxis = new NumberAxis(xLabel);
        plot.setDomainAxis(timeAxis);

        // Create the chart using JFreeChart
        JFreeChart combinedChart = new JFreeChart(title, JFreeChart.DEFAULT_TITLE_FONT, plot, true);

        // Display the chart in a new window
        JFrame frame = new JFrame(title);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new ChartPanel(combinedChart));
        frame.pack();
        frame.setVisible(true);
    }

    /**
     * Helper method to calculate the axis range for a series.
     * 
     * @param series The series for which to calculate the axis range.
     * @return The range to be set for the axis.
     */
    private Range calculateAxisRange(XYSeries series) {
        double min = Double.MAX_VALUE;
        double max = Double.MIN_VALUE;
        for (int i = 0; i < series.getItemCount(); i++) {
            double value = series.getY(i).doubleValue();
            if (value < min) min = value;
            if (value > max) max = value;
        }

        // Add padding to the axis range
        double padding = (max - min) * 0.1;
        return new Range(min - padding, max + padding);
    }
}
