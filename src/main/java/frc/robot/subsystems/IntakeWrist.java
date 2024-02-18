// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.trobot5013lib.CANCoderWrapper;

public class IntakeWrist extends SubsystemBase {

    private final TalonFX intakeWristMotor = new TalonFX(IntakeConstants.INTAKE_WRIST_MOTOR_CAN_ID);
    private CANCoderWrapper encoder = new CANCoderWrapper(IntakeConstants.INTAKE_ENCODER_CAN_ID, true, IntakeConstants.CANCODER_OFFSET_ROTATIONS);
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
    private IntakeRollers m_intakeRollers;

    /** Creates a new IntakeShoulder. */
    public IntakeWrist(IntakeRollers intakeRollers) {
        super();
        this.m_intakeRollers = intakeRollers;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeWristMotor.getConfigurator().apply(config);
        wristController.setTolerance(IntakeConstants.RotationGains.kPositionTolerance.getRadians(), 0.01);
        wristController.enableContinuousInput(0,2*Math.PI);
    }

    public double getAngle() {
        double value = encoder.getAbsPositionRadians() ;
        if (value < 0) {
            value += 2 * Math.PI;
        }
        return value;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeAngle", Math.toDegrees(getAngle()));
        if (this.stop == true) {
            intakeWristMotor.setControl(wristVoltageOut.withOutput(0));
        } 
        else {
            if (wristGoalRadians != wristController.getGoal().position) {
                wristController.setGoal(wristGoalRadians);
            }
            double pidVal = wristController.calculate(getAngle());
            SmartDashboard.putNumber("pidValue", pidVal);
            State setpoint = wristController.getSetpoint();
            double groundRelativeSetpointRadians = getGroundRelativeWristPositionRadians(setpoint.position);
            double acceleration = (wristController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
            double feedforwardVal = feedforward.calculate(getGroundRelativeWristPositionRadians(),wristController.getSetpoint().velocity, acceleration);
            SmartDashboard.putNumber("feedforwardVal",feedforwardVal);
        
            intakeWristMotor.setControl(wristVoltageOut.withOutput(MathUtil.clamp(pidVal + feedforwardVal,-12.0,12.0)));
            lastSpeed = wristController.getSetpoint().velocity;
            lastTime = Timer.getFPGATimestamp();
        }
            SmartDashboard.putNumber("Ground Angle" , Math.toDegrees(getGroundRelativeWristPositionRadians()));
            SmartDashboard.putNumber("Error",wristController.getPositionError());
            SmartDashboard.putNumber("AbsPosition",Math.toDegrees(encoder.getAbsPositionRadians()));
            SmartDashboard.putNumber("Shooter Angle", Math.toDegrees(getLauncherShoulder().getShoulderAngleRadians()));
            SmartDashboard.putNumber("Wrist Cancoder Rotations", encoder.getCanandcoder().getAbsolutePosition().getValueAsDouble() );
    }

    public void deploy() {
        this.stop = false;
        double goal = Math.PI - getLauncherShoulder().getShoulderAngleRadians() - IntakeConstants.DEPLOY_SETPOINT_TO_GROUND;
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

    public Command intakeGamePiece(){
        return deployCommand().andThen(m_intakeRollers.intakeGamepieceCommand()).until(m_intakeRollers::hasGamePiece).andThen(retractCommand());
    }

}
