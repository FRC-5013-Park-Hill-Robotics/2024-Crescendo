// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.trobot5013lib.HeliumEncoderWrapper;

public class LauncherShoulder extends SubsystemBase {

    private final TalonFX launcherShoulderMotor = new TalonFX(LauncherConstants.LAUNCHER_SHOULDER_MOTOR_CAN_ID);
    private final HeliumEncoderWrapper encoder = new HeliumEncoderWrapper(LauncherConstants.LAUNCHER_ENCODER_CAN_ID);
    public double setpointRadians = 0;
    private ArmFeedforward feedforward = new ArmFeedforward(
            LauncherConstants.RotationGains.kS,
            LauncherConstants.RotationGains.kG,
            LauncherConstants.RotationGains.kV, 
            LauncherConstants.RotationGains.kA);
    private Constraints shoulderConstraints = new Constraints(LauncherConstants.RotationGains.kMaxSpeed,
            LauncherConstants.RotationGains.kMaxAcceleration);
    private ProfiledPIDController shoulderController = new ProfiledPIDController(
        LauncherConstants.RotationGains.kP,
        LauncherConstants.RotationGains.kI,
        LauncherConstants.RotationGains.kD,
         shoulderConstraints);
    private final VoltageOut shoulderVoltageOut = new VoltageOut(0);
    private double shoulderGoalRadians = 0;
    private double lastSpeed = 0;
    private double lastTime = 0;

    /** Creates a new IntakeShoulder. */
    public LauncherShoulder() {
        launcherShoulderMotor.getConfigurator().apply(new TalonFXConfiguration());
        launcherShoulderMotor.setInverted(true);
        shoulderController.setTolerance(LauncherConstants.RotationGains.kPositionTolerance.getRadians());
        shoulderController.disableContinuousInput();
    }

    @Override
    public void periodic() {
        double pidVal = shoulderController.calculate(encoder.getAbsPositionRadians(), shoulderGoalRadians);
        State setpoint = shoulderController.getSetpoint();
        double acceleration = (shoulderController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        double feedforwardVal = feedforward.calculate(setpoint.position,shoulderController.getSetpoint().velocity, acceleration);
        //launcherShoulderMotor.setControl(shoulderVoltageOut.withOutput(pidVal + feedforwardVal));
        lastSpeed = shoulderController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    public void retract() {
      setShoulderGoalRadians(LauncherConstants.RETRACT_SETPOINT);
    }

    public void amp() {
      setShoulderGoalRadians(LauncherConstants.AMP_SETPOINT);
    }

    public void setShoulderGoalRadians(double radians) {
      shoulderGoalRadians = radians;
    }

    protected double getShoulderAngleRadians(){
      return encoder.getAbsPositionRadians() + Math.PI/2;
    }

    public Command retractCommand(){
      Command result = run(this::retract).until(shoulderController::atGoal);
      return result;
    }

    public Command ampCommand(){
      Command result = run(this::amp).until(shoulderController::atGoal);
      return result;
    }

}
