// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.LauncherConstants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class LauncherRollers extends SubsystemBase {
  /** Creates a new LauncherRollers. */
  private TalonFX rightMotor = new TalonFX(LauncherConstants.LAUNCHER_RIGHT_CAN_ID);
  private TalonFX leftMotor= new TalonFX(LauncherConstants.LAUNCHER_LEFT_CAN_ID);

  public LauncherRollers() {
      rightMotor.getConfigurator().apply(new TalonFXConfiguration());
      leftMotor.getConfigurator().apply(new TalonFXConfiguration());
      rightMotor.setInverted(true);
      leftMotor.setInverted(false);
  }


  public void stopLauncher(){

  } 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    rightMotor.set(0);
    leftMotor.set(0);
  }
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_rotation = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config( Volts.of(1).per(Seconds.of(1)), Volts.of(7), null,null),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    (Measure<Voltage> volts) -> {
                        rightMotor.setVoltage(volts.in(Volts));
                        leftMotor.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // Record a frame for the wheel motor. 
                        log.motor("right")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_rotation.mut_replace(rightMotor.getPosition().getValueAsDouble(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(rightMotor.getVelocity().getValueAsDouble(), RadiansPerSecond));
                        log.motor("left")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_rotation.mut_replace(leftMotor.getPosition().getValueAsDouble(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(leftMotor.getVelocity().getValueAsDouble(), RadiansPerSecond));

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
}
