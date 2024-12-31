package com.main;

import java.util.ArrayList;

import com.main.RK4Solver.ControlType;

class RK4Solver {
    private final ODEFunction odeFunction;
    private ControlType mode;
    private FFFunction feedForwardFunction;
    private SystemForceFunction systemForcesFunc;
    private ForceMultiplicationFunction forceMultiplication;
    private ArrayList<Double> motorCommands;

    public enum ControlType{
        POSITION,
        VELOCITY
    }

    public RK4Solver(ODEFunction odeFunction) {
        this.odeFunction = odeFunction;
    }

    public RK4Solver( Motor motor, FFFunction ff, SystemForceFunction sff, ForceMultiplicationFunction fmf, ControlType mode ){
        this.mode = mode;
        this.feedForwardFunction = ff;
        this.systemForcesFunc = sff;
        this.forceMultiplication = fmf;
        this.mode = mode;
        
        motorCommands = new ArrayList<>();

        if( null == mode){
            this.odeFunction = (input, target, params, time) -> { return new double[]{0, 0, 0};};
        }
        else switch (mode) {
            case POSITION:
                this.odeFunction = (input, target, params, time) -> {
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
                    double forceMultiplier = forceMultiplication.compute(y); // Apply force multiplier
                    double da = (y - target); // Error term (difference between target and current position)
                    
                    double systemForces = systemForcesFunc.compute(y, b, time);
                    // Calculate motor force, including feedforward and PID control
                    double maxForce = motor.maxTorqueFromVel(b / forceMultiplier);
                    //Find force to cause acceleration of acceleration limit F = ma
                    // motorForce =
                    
                    double motorForce = forceMultiplier * Math.max(-maxForce, Math.min(
                            (-D_ * b - I_ * a - P_ * (y - target) - feedForwardFunction.compute(y, b, time)/forceMultiplier), maxForce));
                    
                    motorCommands.add(motor.controllerSingalFromTorque(motorForce/forceMultiplier, b*forceMultiplier));
                    
                    // Update velocity using Newton's second law
                    double db = (motorForce + systemForces) / m_;
                    // Return the state derivatives
                    return new double[]{dy, da, db};
                };  break;
            case VELOCITY:
                this.odeFunction = (input, target, params, time) -> {
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
                    double forceMultiplier = forceMultiplication.compute(y); // Apply force multiplier
                    double da = (b - target); // Error term (difference between target and current position)
                    
                    double systemForces = systemForcesFunc.compute(y, b, time);
                    // Calculate motor force, including feedforward and PID control
                    double maxForce = motor.maxTorqueFromVel(b / forceMultiplier);
                    //Find force to cause acceleration of acceleration limit F = ma
                    // motorForce =
                    
                    double motorForce = forceMultiplier * Math.max(-maxForce, Math.min(
                            (( - I_ * a - P_ * (b - target))/(1+D_) - feedForwardFunction.compute(y, b, time)/forceMultiplier), maxForce));
                    
                    motorCommands.add(motor.controllerSingalFromTorque(motorForce/forceMultiplier, b*forceMultiplier));
                    
                    // Update velocity using Newton's second law
                    double db = (motorForce + systemForces) / m_;
                    // Return the state derivatives
                    return new double[]{dy, da, db};
                };  break;
            default:
                this.odeFunction = (input, target, params, time) -> { return new double[]{0, 0, 0};};
                break;
        }


    }

    public double[] step(double[] currentState, double target, double[] params, double time, double dt) {
        double[] k1 = odeFunction.compute(currentState, target, params, time);
        double[] k2 = odeFunction.compute(add(currentState, mult(k1, dt / 2)), target, params, time + dt/2);
        double[] k3 = odeFunction.compute(add(currentState, mult(k2, dt / 2)), target, params, time + dt/2);
        double[] k4 = odeFunction.compute(add(currentState, mult(k3, dt)), target, params, time + dt);

        return add(currentState, mult(add(k1, mult(k2, 2), mult(k3, 2), k4), dt / 6));
    }

    public double[][] runSimulation(double[] initialState, double[] targets, double[] params, double startTime, double endTime, double dt) {
        int steps = (int) Math.ceil((endTime - startTime) / dt);
        double[][] results = new double[steps][];
        results[0] = initialState;

        for (int i = 1; i < steps; i++) {
            double target = targets[i - 1];
            results[i] = step(results[i - 1], target, params, i *dt, dt);
        }

        return results;
    }

    private double[] add(double[]... arrays) {
        int length = arrays[0].length;
        double[] result = new double[length];
        for (double[] array : arrays) {
            for (int i = 0; i < length; i++) {
                result[i] += array[i];
            }
        }
        return result;
    }

    private double[] mult(double[] arr, double scalar) {
        double[] result = new double[arr.length];
        for (int i = 0; i < arr.length; i++) {
            result[i] = arr[i] * scalar;
        }
        return result;
    }

    public ArrayList<Double> getMotorCommands(){
        return motorCommands;
    }
}

@FunctionalInterface
interface ODEFunction {
    double[] compute(double[] state, double target, double[] params, double time);
}

@FunctionalInterface
interface FFFunction{
    double compute(double position, double velocity, double time);
}

@FunctionalInterface
interface SystemForceFunction{
    double compute(double position, double velocity, double time);
}

@FunctionalInterface
interface ForceMultiplicationFunction{
    double compute(double position);
}

