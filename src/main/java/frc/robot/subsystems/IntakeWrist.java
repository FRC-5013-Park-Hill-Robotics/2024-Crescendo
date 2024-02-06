// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.trobot5013lib.HeliumEncoderWrapper;

public class IntakeWrist extends SubsystemBase {

    private final TalonFX intakeWristMotor = new TalonFX(IntakeConstants.INTAKE_WRIST_MOTOR_CAN_ID);
    private final HeliumEncoderWrapper encoder = new HeliumEncoderWrapper(IntakeConstants.INTAKE_ENCODER_CAN_ID);
    public double setpointRadians = 0;
    private ArmFeedforward feedforward = new ArmFeedforward(
            IntakeConstants.RotationGains.kS,
            IntakeConstants.RotationGains.kG,
            IntakeConstants.RotationGains.kV, 
            IntakeConstants.RotationGains.kA);
    private Constraints wristConstraints = new Constraints(IntakeConstants.RotationGains.kMaxSpeed,
            IntakeConstants.RotationGains.kMaxAcceleration);
    private ProfiledPIDController wristController = new ProfiledPIDController(
        IntakeConstants.RotationGains.kP,
        IntakeConstants.RotationGains.kI,
        IntakeConstants.RotationGains.kD,
         wristConstraints);
    private final TorqueCurrentFOC wrisTorqueCurrentFOC = new TorqueCurrentFOC(0);
    private double wristGoalRadians = 0;
    private double lastSpeed = 0;
    private double lastTime = 0;

    /** Creates a new IntakeShoulder. */
    public IntakeWrist() {
        wristController.setTolerance(IntakeConstants.RotationGains.kPositionTolerance.getRadians());
        wristController.disableContinuousInput();
    }

    @Override
    public void periodic() {
        double pidVal = wristController.calculate(encoder.getAbsPositionRadians(), wristGoalRadians);
        State setpoint = wristController.getSetpoint();
        double groundRelativeSetpointRadians = getGroundRelativeWristPositionRadians(setpoint.position);
        double acceleration = (wristController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        double feedforwardVal = feedforward.calculate(groundRelativeSetpointRadians,wristController.getSetpoint().velocity, acceleration);
    //    intakeWristMotor.setControl(wrisTorqueCurrentFOC.withOutput(pidVal + feedforwardVal));
        lastSpeed = wristController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    public void deploy() {
        double goal = Math.PI - IntakeConstants.DEPLOY_SETPOINT_TO_GROUND
                - getLauncherShoulder().getShoulderAngleRadians();
        setWristGoalRadians(goal);
    }

    public void retract() {
        setWristGoalRadians(IntakeConstants.RETRACT_SETPOINT);
    }

    public void setWristGoalRadians(double radians) {
        wristGoalRadians = radians;
    }

    private LauncherShoulder getLauncherShoulder() {
        return RobotContainer.getInstance().getLauncherShoulder();
    }

    protected double getGroundRelativeWristPositionRadians(double launcherRelativeAngleRadians) {
        return Math.PI - getLauncherShoulder().getShoulderAngleRadians() + launcherRelativeAngleRadians;
    }

    public double getGroundRelativeWristPositionRadians(){
        return getGroundRelativeWristPositionRadians (encoder.getAbsPositionRadians()) ;
    }

    public Command deployCommand(){
        Command result = run(this::deploy).until(wristController::atGoal);
        result.addRequirements(getLauncherShoulder());
        return result;
    }

    public Command retractCommand(){
        Command result = run(this::retract).until(wristController::atGoal);
        result.addRequirements(getLauncherShoulder());
        return result;
    }

}
