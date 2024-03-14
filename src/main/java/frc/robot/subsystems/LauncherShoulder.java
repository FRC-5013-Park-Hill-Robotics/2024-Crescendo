// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.trobot5013lib.RevThroughBoreEncoder;
import frc.robot.trobot5013lib.TrobotUtil;

public class LauncherShoulder extends SubsystemBase {

    private final TalonFX launcherShoulderMotor = new TalonFX(LauncherConstants.LAUNCHER_SHOULDER_MOTOR_CAN_ID);
    private final RevThroughBoreEncoder encoder = new RevThroughBoreEncoder(LauncherConstants.ENCODER_DIO_PORT, false, LauncherConstants.OFFSET_RADIANS);
    public double setpointRadians = 0;
    private ArmFeedforward feedforward = new ArmFeedforward(
            LauncherConstants.RotationGains.kS,
            LauncherConstants.RotationGains.kG,
            LauncherConstants.RotationGains.kV, 
            LauncherConstants.RotationGains.kA);
    //Because of the use of gas shocks the arm is not as simple as a normal feed forward, so we are making a ff for the higher angle
    //we will then interpolate between that and the lower angle
    private ArmFeedforward feedforwardHigh = new ArmFeedforward(
            LauncherConstants.RotationGains.kS,
            LauncherConstants.RotationGains.kGHhigh,
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
    private double shoulderGoalRadians = LauncherConstants.START_ANGLE_RADIANS;
    private double lastSpeed = 0;
    private double lastTime = 0;

    /** Creates a new IntakeShoulder. */
    public LauncherShoulder() {
        launcherShoulderMotor.getConfigurator().apply(new TalonFXConfiguration());
        launcherShoulderMotor.setInverted(false);
        shoulderController.setTolerance(LauncherConstants.RotationGains.kPositionTolerance.getRadians());
        shoulderController.enableContinuousInput(0,2*Math.PI);
        setShoulderGoalRadians(getShoulderAngleRadians());
    }

    public boolean atGoal(){
      return TrobotUtil.withinTolerance(getShoulderAngleRadians(), shoulderGoalRadians, LauncherConstants.RotationGains.kPositionTolerance.getRadians());
    }

    @Override
    public void periodic() {
      
        if(shoulderGoalRadians > LauncherConstants.SHOULDER_ANGLE_MAX){
          shoulderGoalRadians = LauncherConstants.SHOULDER_ANGLE_MAX;
        }
        if(shoulderGoalRadians < LauncherConstants.SHOULDER_ANGLE_MIN){
          shoulderGoalRadians = LauncherConstants.SHOULDER_ANGLE_MIN;
        }

        double pidVal = shoulderController.calculate(getShoulderAngleRadians(), shoulderGoalRadians);
        State setpoint = shoulderController.getSetpoint();
        double acceleration = (shoulderController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        double feedforwardVal = feedforward.calculate(setpoint.position,shoulderController.getSetpoint().velocity, acceleration);
        double feedforwardValhigh = feedforwardHigh.calculate(Math.PI/2 - setpoint.position,shoulderController.getSetpoint().velocity, acceleration);
        double combinedFF = MathUtil.interpolate(feedforwardVal,feedforwardValhigh, setpoint.position/Math.PI);

        double output = 0;
        if(pidVal + combinedFF > 0 && LauncherConstants.SHOULDER_ANGLE_MAX < getShoulderAngleRadians()){
          output = 0;
        } 
        else if (pidVal + combinedFF < 0 && LauncherConstants.SHOULDER_ANGLE_MIN > getShoulderAngleRadians()){
          output = 0;
        } else{
          output = pidVal + combinedFF;
        }
        launcherShoulderMotor.setControl(shoulderVoltageOut.withOutput(output));
      
        lastSpeed = shoulderController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("Launcher Goal", Math.toDegrees(shoulderGoalRadians));
        SmartDashboard.putNumber("Shoulder Absolute" , encoder.getAngle().getDegrees());
        SmartDashboard.putNumber("Shoulder Ground Relative" , Math.toDegrees(getShoulderAngleRadians()));
        SmartDashboard.putBoolean("Launcher at goal", atGoal());
    
      }

    public void retract() {
      setShoulderGoalRadians(LauncherConstants.RETRACT_SETPOINT);
    }

    public void setShoulderGoalRadians(double radians) {
      shoulderGoalRadians = radians;
    }

    protected double getShoulderAngleRadians(){
      return (encoder.getAngle().getRadians());
    }

    public void ampAngle(){
        double goal = LauncherConstants.AMP_ANGLE_RADANS;
        setShoulderGoalRadians(goal);
    }
    public Command ampAngleCommand(){
        Command result = run(this::ampAngle).until(this::atAmp);
        return result;
    }

    public boolean atAmp(){
      return getShoulderAngleRadians() >= LauncherConstants.AMP_ANGLE_RADANS - LauncherConstants.RotationGains.kPositionTolerance.getRadians() &&
      getShoulderAngleRadians() <= LauncherConstants.AMP_ANGLE_RADANS + LauncherConstants.RotationGains.kPositionTolerance.getRadians();
    }
    public Command retractCommand(){
      Command result = run(this::retract).until(shoulderController::atGoal);
      return result;
    }

    public Command goToSetpointCommand(double radians) {
      Command result = runOnce(() -> setShoulderGoalRadians(radians));
      return result;
    }

    public Command goToSetpointUntilCompleteCommand(double radians) {
      Command result = run(() -> setShoulderGoalRadians(radians)).until(this::atGoal);
      return result;
    }

    public void incrementAngle(double radianChange) {
      this.shoulderGoalRadians += radianChange;
  
    }
  
    public Command incrementAngleCommand(double radianChange){
      Command result = runOnce(()-> incrementAngle(radianChange));
      return result;
    } 

    public Command holdCommand(){
      return runOnce(() -> setShoulderGoalRadians(LauncherConstants.START_ANGLE_RADIANS));
    }
}
