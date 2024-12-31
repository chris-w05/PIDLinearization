package com.main;

import java.io.IOException;

/**
 * Main class that runs the simulation for a motor control system.
 * This class demonstrates the use of a Runge-Kutta 4th order (RK4) solver
 * to simulate a system's behavior with PID control, including force calculations
 * based on position, velocity, and a motor's response.
 */
public class Main {
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
        double endTime = 20.0;  // End time for the simulation
        double dt = 0.0001;      // Time step for the simulation
        double[] initialConditions = {0, 0, 0}; // Initial conditions: position, error, and velocity

        // Set up the target trajectory for the simulation (e.g., a position setpoint)
        double[] targets = new double[(int) Math.ceil((endTime - startTime) / dt)];
        for (int i = 0; i < targets.length; i++) {
            if (i < targets.length / 4) {
                targets[i] = 10; // Initial target value
            }
            else if (i < targets.length / 2) {
                targets[i] = -10; // Initial target value
            }
             else {
                targets[i] = 0; // Target value transitions
            }
        }

        // PID constants (adjust these values to tune the controller)
        double P = .5;  // Proportional constant (force/position)
        double I = .00;  // Integral constant (force/accumulated error)
        double D = .4;   // Derivative constant (force/velocity)
        double m = 10;     // Mass of the system

    
        // Motor settings
        Motor motor = MotorLoader.loadMotorByName("Kraken X60 (FOC)*"); // Load motor by name
        motor.setCurrentLimit(40.0); // Set current limit for the motor

        //System functions:
        FFFunction feedForward = (position, velocity, time) -> { return -9.81 * 10; };
        SystemForceFunction systemForceFunc = (position, velocity, time) -> { return -9.81 * 10;};
        ForceMultiplicationFunction forceMult = (position) -> {return 100;};


        // Create the RK4 solver with the control system dynamics
        RK4Solver solver3 = new RK4Solver( P, I, D, m, motor, feedForward, systemForceFunc, forceMult, RK4Solver.ControlType.POSITION);
        

        // Run the simulation using the RK4 solver
        double[][] results = solver3.runSimulation(initialConditions, targets, startTime, endTime, dt);

        // Create and display the simulation results using charts
        ChartPlotter plotter = new ChartPlotter();
        plotter.createCombinedChart("System Simulation Results", "Time (s)", results, startTime, dt, true, false, false, false);
        plotter.createCommandChart("Motor commmands over time", "time", "Motor command value", solver3.getMotorCommands(), 0.0, dt);
        // double[] forces = new double[targets.length];
        // for(int i = 0; i < forces.length; i ++){
        //     forces[i] = systemForces( results[i][0], results[i][2], i * dt);
        // }

        //plotter.createChart("Forces vs time", "Time", "Force", forces , 0.0, dt);
        // Additional individual charts can be created here for position, error, velocity, etc.
    }
}
