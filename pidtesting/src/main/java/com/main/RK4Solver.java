package com.main;

import java.util.ArrayList;


class RK4Solver {
    private final ODEFunction odeFunction;
    

    private ControlType mode;
    private FFFunction feedForwardFunction;
    private SystemForceFunction systemForcesFunc;
    private ForceMultiplicationFunction forceMultiplication;
    private ArrayList<Double> motorCommands;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double mass = 1;
    private Motor motor;

    private final ODEFunction positionODE = (input, target, time) -> {
                    // Unpack the state variables
                    double y = input[0]; // Position
                    double a = input[1]; // Error (accumulated)
                    double b = input[2]; // Velocity
                    
                    // Unpack the PID and system parameters
                    
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
                            (-this.kD * b - this.kI * a - this.kP * (y - target) - feedForwardFunction.compute(y, b, time)/forceMultiplier), maxForce));
                    
                    motorCommands.add(motor.controllerSingalFromTorque(motorForce/forceMultiplier, b*forceMultiplier));
                    
                    // Update velocity using Newton's second law
                    double db = (motorForce + systemForces) / this.mass;
                    // Return the state derivatives
                    return new double[]{dy, da, db};
    };

    private final ODEFunction velocityODE = (input, target, time) -> {
                    // Unpack the state variables
                    double y = input[0]; // Position
                    double a = input[1]; // Error (accumulated)
                    double b = input[2]; // Velocity
                    
                    
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
                            (( - this.kI * a - this.kP * (b - target))/(1+this.kD) - feedForwardFunction.compute(y, b, time)/forceMultiplier), maxForce));
                    
                    motorCommands.add(motor.controllerSingalFromTorque(motorForce/forceMultiplier, b*forceMultiplier));
                    
                    // Update velocity using Newton's second law
                    double db = (motorForce + systemForces) / this.mass;
                    // Return the state derivatives
                    return new double[]{dy, da, db};
                };

    public enum ControlType{
        POSITION,
        VELOCITY
    }

    public RK4Solver(ODEFunction odeFunction) {
        this.odeFunction = odeFunction;
    }

    public RK4Solver( double kP, double kI, double kD, double mass, Motor motor, FFFunction ff, SystemForceFunction sff, ForceMultiplicationFunction fmf, ControlType mode ){
        this.mode = mode;
        this.feedForwardFunction = ff;
        this.systemForcesFunc = sff;
        this.forceMultiplication = fmf;
        this.mode = mode;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.mass = mass;
        this.motor = motor;
        
        motorCommands = new ArrayList<>();

        if( null == mode){
            this.odeFunction = (input, target, time) -> { return new double[]{0, 0, 0};};
        }
        else switch (mode) {
            case POSITION:
                this.odeFunction = positionODE;
                break;
            case VELOCITY:
                this.odeFunction = velocityODE;
                break;
            default:
                this.odeFunction = (input, target, time) -> { return new double[]{0, 0, 0};};
                break;
        }
    }

    public RK4Solver( Motor motor, FFFunction ff, SystemForceFunction sff, ForceMultiplicationFunction fmf, ControlType mode ){
        this.mode = mode;
        this.feedForwardFunction = ff;
        this.systemForcesFunc = sff;
        this.forceMultiplication = fmf;
        this.mode = mode;
        
        motorCommands = new ArrayList<>();

        if( null == mode){
            this.odeFunction = (input, target, time) -> { return new double[]{0, 0, 0};};
        }
        else switch (mode) {
            case POSITION:
                this.odeFunction = positionODE;
                break;
            case VELOCITY:
                this.odeFunction = velocityODE;
                break;
            default:
                this.odeFunction = (input, target, time) -> { return new double[]{0, 0, 0};};
                break;
        }
    }

    public void setParams(double kP, double kI, double kD, double mass){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.mass = mass;
    }

    public void setMode( ControlType mode){
        this.mode = mode;
    }

    public double[] step(double[] currentState, double target, double time, double dt) {
        double[] k1 = odeFunction.compute(currentState, target, time);
        double[] k2 = odeFunction.compute(add(currentState, mult(k1, dt / 2)), target, time + dt/2);
        double[] k3 = odeFunction.compute(add(currentState, mult(k2, dt / 2)), target, time + dt/2);
        double[] k4 = odeFunction.compute(add(currentState, mult(k3, dt)), target, time + dt);

        return add(currentState, mult(add(k1, mult(k2, 2), mult(k3, 2), k4), dt / 6));
    }

    public double[][] runSimulation(double[] initialState, double[] targets, double startTime, double endTime, double dt) {
        int steps = (int) Math.ceil((endTime - startTime) / dt);
        double[][] results = new double[steps][];
        results[0] = initialState;

        for (int i = 1; i < steps; i++) {
            double target = targets[i - 1];
            results[i] = step(results[i - 1], target, i *dt, dt);
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
    double[] compute(double[] state, double target, double time);
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

