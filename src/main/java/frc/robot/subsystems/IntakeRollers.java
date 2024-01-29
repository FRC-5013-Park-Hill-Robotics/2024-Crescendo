// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {

    // private Motor intakeMotor = new Motor(ID, MotorType);

    public IntakeRollers() {
    }

    public void feedIn() {
        //intakeMotor.set(IntakeConstants.intakeMotorSpeed);
    }

    public void feedOut() {
        //intakeMotor.set(-IntakeConstants.intakeMotorSpeed);
    }

    public void stop() {
        //intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
