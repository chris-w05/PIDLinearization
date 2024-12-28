package com.main;

import java.io.IOException;
import java.util.ArrayList;

/**
 * Main class that runs the simulation for a motor control system.
 * This class demonstrates the use of a Runge-Kutta 4th order (RK4) solver
 * to simulate a system's behavior with PID control, including force calculations
 * based on position, velocity, and a motor's response.
 */
public class Main {

    /**
     * Calculates the feedforward term based on the given position (y).
     * The feedforward term represents a physical force based on gravitational acceleration and position.
     * This is a force requested from the motor dependent on the position (not proportional to position).
     * The intention is that the feedForward function should calcel the external forces on the system, leaving
     * a nice linear space for the PID control to live.
     * 
     * @param y The position of the system, used to calculate the feedforward force.
     * @return The computed feedforward force.
     */
    private static double feedForwardFunction(double y) {
        // return 0.0;
        return -9.81 * 10 * .5 * Math.cos(y); // Gravitational force
    }

    /**
     * Calculates the total forces acting on the system based on position, velocity, and mass.
     * This method includes gravitational forces dependent on the position. This is everything putting forces on the system
     * outside of the motor. (Friction, vibration, gravity, another robot, etc)
     *
     * @param position The current position of the system.
     * @param velocity The current velocity of the system.
     * @param mass The mass of the system, affecting the gravitational force.
     * @return The net force acting on the system.
     */
    private static double systemForces(double position, double velocity, double mass) {
        // Gravity force based on position (assuming vertical motion)
        return -9.81 * mass * .5 * Math.cos(position);
        // return -9.81 * mass;
    }

    /**
     * Multiplies the force based on the current position.
     * This function could represent an additional scaling factor or a system-specific adjustment.
     * This is used to model things such as linkages, gear ratios, or anything that uses conservation of
     * energy between the input and output of the system.
     * 
     * Also note that this also is what handles the unit conversion between the output velocity(units depend on the scenario) and input velocity(rpm)
     * 
     * @param position The position of the system, used to calculate the scaling factor.
     * @return The force multiplier based on the position.
     */
    private static double forceMultiplication(double position) {
        return 100; // Arbitrary scaling factor for the force based on position
    }

    /**
     * Main method that sets up and runs the motor control system simulation.
     * It initializes the PID constants, motor settings, and the RK4 solver to simulate system behavior
     * and generates plots of the simulation results.
     * 
     * @param args Command-line arguments, not used in this implementation.
     * @throws IOException If there is an issue loading the motor or plotting the chart.
     */
    public static void main(String[] args) throws IOException {
        // Simulation settings
        double startTime = 0.0; // Start time for the simulation
        double endTime = 10.0;  // End time for the simulation
        double dt = 0.0001;      // Time step for the simulation
        double[] initialConditions = {0, 0, 0}; // Initial conditions: position, error, and velocity

        // Set up the target trajectory for the simulation (e.g., a position setpoint)
        double[] targets = new double[(int) Math.ceil((endTime - startTime) / dt)];
        for (int i = 0; i < targets.length; i++) {
            if (i < targets.length / 4) {
                targets[i] = 4; // Initial target value
            } else {
                targets[i] = 0; // Target value transitions
            }
        }

        // PID constants (adjust these values to tune the controller)
        double P = 6000;  // Proportional constant (force/position)
        double I = 100;  // Integral constant (force/accumulated error)
        double D = 300;   // Derivative constant (force/velocity)
        double m = 1;     // Mass of the system

        // Motor settings
        Motor motor = MotorLoader.loadMotorByName("Kraken X60 (FOC)*"); // Load motor by name
        double maxMotorVoltage = 12; // Maximum voltage for the motor
        motor.setCurrentLimit(40.0); // Set current limit for the motor
        ArrayList<Double> motorCommands = new ArrayList<Double>();

        int count = 0;

        // Create the RK4 solver with the control system dynamics
        RK4Solver solver = new RK4Solver((input, target, params) -> {
            // Unpack the state variables
            double y = input[0]; // Position
            double a = input[1]; // Error (accumulated)
            double b = input[2]; // Velocity

            // Unpack the PID and system parameters
            double P_ = params[0];
            double I_ = params[1];
            double D_ = params[2];
            double m_ = params[3];

            // State equations for the system
            double dy = b; // Position derivative is the velocity
            double forceMultiplier = forceMultiplication(y); // Apply force multiplier
            double da = (y - target); // Error term (difference between target and current position)
            
            // Calculate motor force, including feedforward and PID control
            double maxForce = motor.maxTorqueFromVel(b / forceMultiplier); 
            double motorForce = forceMultiplier * Math.max(-maxForce, Math.min(
                (-D_ * b - I_ * a - P_ * (y - target) - feedForwardFunction(y)), maxForce));
            
            motorCommands.add(motor.controllerSingalFromTorque(motorForce/forceMultiplier, b*forceMultiplier));
            
            // Update velocity using Newton's second law
            double db = (motorForce + systemForces(y, b, m_)) / m_;
            // Return the state derivatives
            return new double[]{dy, da, db};
        });


        // Run the simulation using the RK4 solver
        double[][] results = solver.runSimulation(initialConditions, targets, new double[]{P, I, D, m}, startTime, endTime, dt);

        // Create and display the simulation results using charts
        ChartPlotter plotter = new ChartPlotter();
        plotter.createCombinedChart("System Simulation Results", "Time (s)", results, startTime, dt, true, false, false, false);
        plotter.createCommandChart("Motor commmands over time", "time", "Motor command value", motorCommands, 0.0, dt);
        // Additional individual charts can be created here for position, error, velocity, etc.
    }
}
