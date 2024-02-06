// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sysID;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;

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
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.LauncherShoulder;
import frc.robot.trobot5013lib.HeliumEncoderWrapper;

public class IntakeWristId extends SubsystemBase {

    private final TalonFX intakeWristMotor = new TalonFX(IntakeConstants.INTAKE_WRIST_MOTOR_CAN_ID);
    private final HeliumEncoderWrapper encoder = new HeliumEncoderWrapper(IntakeConstants.INTAKE_ENCODER_CAN_ID);
    public double setpointRadians = 0;
    private ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.RotationGains.kS, IntakeConstants.RotationGains.kG,
            IntakeConstants.RotationGains.kV, IntakeConstants.RotationGains.kA);

    /** Creates a new IntakeShoulder. */
    public IntakeWristId() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double offsetPosition(){
        return encoder.getAbsPositionRadians() - (2 * Math.PI * 0.055);
    }
//.055
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_rotation = mutable(Radians.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));
     private final TorqueCurrentFOC m_torqueCurrentFOC = new TorqueCurrentFOC(0);
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config( Volts.of(2).per(Seconds.of(1)), Volts.of(20), null,null),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    (Measure<Voltage> volts) -> {
                        intakeWristMotor.setControl(m_torqueCurrentFOC.withOutput(volts.in(Volts)));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // Record a frame for the wrist motor. 
                        log.motor("wrist")
                                .voltage(
                                        m_appliedVoltage.mut_replace(intakeWristMotor.getTorqueCurrent().getValueAsDouble()
                                                , Volts))
                                .angularPosition(m_rotation.mut_replace(offsetPosition(), Radians))
                                .angularVelocity(
                                        m_velocity.mut_replace(encoder.getVelocityRadians(), RadiansPerSecond));

                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("IntakeWrist")
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}