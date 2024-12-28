package com.main;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import com.fasterxml.jackson.databind.ObjectMapper;

public class MotorLoader {
    public static List<Motor> loadMotors() throws IOException {
        Path currentPath = Paths.get("").toAbsolutePath();
        String filePath = currentPath + "/pidtesting/src/main/java/com/main/motors.json";
        ObjectMapper mapper = new ObjectMapper();
        return mapper.readValue(new File(filePath), mapper.getTypeFactory().constructCollectionType(List.class, Motor.class));
    }

    public static Motor loadMotorByName( String motorName) throws IOException {
        List<Motor> motors = loadMotors();
        Motor output = motors.stream()
                     .filter(motor -> motor.getName().equalsIgnoreCase(motorName))
                     .findFirst().get();
        double kT = output.getStallTorque().getMagnitude()/output.getStallCurrent().getMagnitude();
        double kV = output.getFreeSpeed().getMagnitude()/12;
        output.setKT(kT); 
        output.setKV(kV);      //ASSUMES A 12V DYNO
        //System.out.println(kT + " " + kV);
        return output;
    }
}