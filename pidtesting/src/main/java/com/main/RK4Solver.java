package com.main;

class RK4Solver {
    private final ODEFunction odeFunction;

    public RK4Solver(ODEFunction odeFunction) {
        this.odeFunction = odeFunction;
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
}

@FunctionalInterface
interface ODEFunction {
    double[] compute(double[] state, double target, double[] params, double time);
}