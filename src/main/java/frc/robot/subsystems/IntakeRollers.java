// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;


import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class IntakeRollers extends SubsystemBase {

    private final TalonFX m_intakeRoller = new TalonFX(CANConstants.INTAKE_ROLLER_ID);
    private ArmFeedforward m_intakFeedforward = new ArmFeedforward(0, 0, 0);
    //Create Feed Forward controller for velocity control using IntakeConstants.RollerGains
    //Create Control Request for Motor of tpe VelocityTorqueCurrentFOC
    //add time of flight sensor for game piece intake/outtake detection

    public IntakeRollers() {
        //Clear motor configs - config facgtory default
        //set motor configs, 
            //PID slot 0, 
            //inversion.
            //idle mode brake.
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

    public boolean hasGamePiece(){
        //TODO implement sensor check for game piece
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //PID calculate
        //Feed Forward Calculate
        //Set motor output
    }
    //TODO update characterization routine for the motor created. Uncomment below code and fix broken motor reference to the motorcontroller you create above.
/*

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe rotational distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Angle> m_rotation = mutable(Rotations.of(0));
  // Mutable holder for unit-safe rotational  velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  private final TorqueCurrentFOC m_torqueCurrentFOC = new TorqueCurrentFOC(0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Using amperage, since it is converted when it comes out it ok.  Config expecst type volts.
      new SysIdRoutine.Config(Volts.of(5).per(Seconds.of(1)), Volts.of(30), null, null),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            rightMotor.setControl(m_torqueCurrentFOC.withOutput(volts.in(Volts)));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the wheel motor.
            log.motor("wheel")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_rotation.mut_replace(rightMotor.getPosition().getValueAsDouble(), Rotations))
                .angularVelocity(
                    m_velocity.mut_replace(rightMotor.getVelocity().getValueAsDouble(), RadiansPerSecond));

          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("LauncherRollers")
          this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
   */
}
