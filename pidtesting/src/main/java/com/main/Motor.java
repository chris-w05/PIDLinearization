package com.main;

import java.io.IOException;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class Motor {
    // Units
    private double kT;            // Nm/A
    private double kV;            // radians/(second*volt)
    private final double iFree;         // Amp
    private final double resistance;    // Ohm
    private double currentLimit = 40;        // Amp

    // Additional fields for JSON
    private String name;
    private Measurement freeSpeed;
    private Measurement stallTorque;
    private Measurement stallCurrent;
    private Measurement freeCurrent;
    private Measurement weight;
    private Measurement diameter;
    private String url;
    private Measurement controllerWeight;


    // Constructor to initialize Motor object
    public Motor(double kT, double kV, double iFree, double resistance) {
        this.kT = kT;
        this.kV = kV;
        this.iFree = iFree;
        this.resistance = resistance;
    }

    // Constructor for Jackson JSON parsing
    public Motor() {
        this.kT = 0;
        this.kV = 0;
        this.iFree = 0;
        this.resistance = 0;
    }

    public void setCurrentLimit(double limit) {
        this.currentLimit = limit;
    }

    public double torqueFromCurrent(double amperage) {
        return kT * (amperage - iFree);
    }

    //Deprecated
    public double torqueFromVoltage(double v_applied, double velocity) {
        //System.out.println( kT + " " + v_applied + " " + velocity + " " + kV + " " + resistance + " " + currentLimit + " " + iFree);

        return kT * (Math.min((v_applied - velocity / kV) / resistance, currentLimit) - iFree);
    }

    public double torqueFromVel(double velocity){
        return  kT*maxCurrentFromVel(velocity);
    }

    public double maxCurrentFromVel(double velocity){
        return stallCurrent.getMagnitude() + velocity*(freeCurrent.getMagnitude() - stallCurrent.getMagnitude())/freeSpeed.getMagnitude();
    }

    public static void main(String[] args) throws IOException{
        //Debugging Torque curve
        double maxV = 10000;
        int numberPoints = 100;
        XYSeries series = new XYSeries("T vs V");
        Motor falcon500 = MotorLoader.loadMotorByName("Kraken X60 (FOC)*");
        falcon500.setCurrentLimit(100);
        for( int i = 0; i < numberPoints; i ++ ){
            double velocity = maxV * i / numberPoints;
            series.add( velocity, falcon500.torqueFromVel( velocity));
        }
        XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(series);

        JFreeChart chart = ChartFactory.createXYLineChart(
                "T vs V",
                "Velocity",
                "Torque",
                dataset
        );

        JFrame frame = new JFrame();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new ChartPanel(chart));
        frame.pack();
        frame.setVisible(true);

    }


    // Getters for the fields
    public String getName() {
        return name;
    }

    public Measurement getFreeSpeed() {
        return freeSpeed;
    }

    public Measurement getStallTorque() {
        return stallTorque;
    }

    public Measurement getStallCurrent() {
        return stallCurrent;
    }

    public Measurement getFreeCurrent() {
        return freeCurrent;
    }

    public Measurement getWeight() {
        return weight;
    }

    public Measurement getDiameter() {
        return diameter;
    }

    public String getUrl() {
        return url;
    }

    public Measurement getControllerWeight() {
        return controllerWeight;
    }

    public void setKT(double value){
        this.kT = value;
    }

    public void setKV(double value){
        this.kV = value;
    }

    // Inner class for units
    public static class Measurement {
        private double magnitude;
        private String unit;

        // Getters
        public double getMagnitude() {
            return magnitude;
        }

        public String getUnit() {
            return unit;
        }
    }
}
