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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private final VoltageOut wristVoltageOut = new VoltageOut(0);
    private double wristGoalRadians = 0;
    private double lastSpeed = 0;
    private double lastTime = 0;

    private boolean stop = true;

    /** Creates a new IntakeShoulder. */
    public IntakeWrist() {
        intakeWristMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeWristMotor.setInverted(true);
        wristController.setTolerance(IntakeConstants.RotationGains.kPositionTolerance.getRadians());
        wristController.disableContinuousInput();

    }

    public double getAngle() {
        return encoder.getAbsPositionRadians() - (2 * Math.PI * 0.476);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeAngle", getAngle());
        if (this.stop == true) {
            intakeWristMotor.setControl(wristVoltageOut.withOutput(0));
        } 
        else {
            double pidVal = wristController.calculate(getAngle(), wristGoalRadians);
            State setpoint = wristController.getSetpoint();
            double groundRelativeSetpointRadians = getGroundRelativeWristPositionRadians(setpoint.position);
            double acceleration = (wristController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
            double feedforwardVal = feedforward.calculate(groundRelativeSetpointRadians,wristController.getSetpoint().velocity, acceleration);
            intakeWristMotor.setControl(wristVoltageOut.withOutput(pidVal + feedforwardVal));
            lastSpeed = wristController.getSetpoint().velocity;
            lastTime = Timer.getFPGATimestamp();
        }
    }

    public void deploy() {
        this.stop = false;
       double goal = Math.PI - IntakeConstants.DEPLOY_SETPOINT_TO_GROUND;
       //         - getLauncherShoulder().getShoulderAngleRadians();
        setWristGoalRadians(goal);
    }

    public void retract() {
        this.stop = false;
        setWristGoalRadians(IntakeConstants.RETRACT_SETPOINT);
    }

    public void stop() {
        this.stop = true;
    }

    public void setWristGoalRadians(double radians) {
        wristGoalRadians = radians;
    }

    private LauncherShoulder getLauncherShoulder() {
        return RobotContainer.getInstance().getLauncherShoulder();
    }

    protected double getGroundRelativeWristPositionRadians(double launcherRelativeAngleRadians) {
        return Math.PI - getLauncherShoulder().getShoulderAngleRadians() - launcherRelativeAngleRadians;
    }

    public double getGroundRelativeWristPositionRadians(){
        return getGroundRelativeWristPositionRadians (getAngle()) ;
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

    public Command stopCommand() {
        Command result = runOnce(this::stop);
        return result;
    }

}
