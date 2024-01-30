// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

/** Add your docs here. */
public class HeliumEncoderWrapper {
    private Canandcoder encoder; 

    public HeliumEncoderWrapper () {

    }


    public HeliumEncoderWrapper (int canID) {
        this.encoder = new Canandcoder(canID);
    }

    public double getAbsPositionRadians() {
        return rotationsToRadians(encoder.getAbsPosition());
    }

    private double rotationsToRadians(double rotations) {
        return 2 * Math.PI * rotations;
    }
    public double getVelocityRadians() {
        return rotationsToRadians(2 * Math.PI * encoder.getVelocity());
    }

    public Canandcoder getCanandcoder() {
        return this.encoder;
    }
    public void setCanandcoder(Canandcoder encoder) {
        this.encoder = encoder;
    }
}
