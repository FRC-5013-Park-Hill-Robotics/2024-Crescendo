// Copybottom (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeConstants;
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
  private TalonFX bottomMotor = new TalonFX(LauncherConstants.LAUNCHER_TOP_CAN_ID);
  private TalonFX topMotor= new TalonFX(LauncherConstants.LAUNCHER_BOTTOM_CAN_ID);
  private VelocityVoltage m_BottomVoltage = new VelocityVoltage(0);
  private VelocityVoltage m_topVoltage = new VelocityVoltage(0);
  private double goalSpeedRPS = 0;

  public LauncherRollers() {
      TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
      bottomConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      bottomConfig.Slot0.kP = LauncherConstants.RollerGains.kP;
      bottomConfig.Slot0.kI = LauncherConstants.RollerGains.kI;
      bottomConfig.Slot0.kD = LauncherConstants.RollerGains.kD;
      bottomConfig.Slot0.kS = LauncherConstants.RollerGains.kS;
      bottomConfig.Slot0.kV = LauncherConstants.RollerGains.kV;
      bottomConfig.Slot0.kA = LauncherConstants.RollerGains.kA;
      bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      bottomMotor.set(0);
      bottomMotor.getConfigurator().apply(bottomConfig);
      m_BottomVoltage.withSlot(0);
      

      TalonFXConfiguration topConfig = new TalonFXConfiguration();
      topConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      topConfig.Slot0.kP = LauncherConstants.RollerGains.kP;
      topConfig.Slot0.kI = LauncherConstants.RollerGains.kI;
      topConfig.Slot0.kD = LauncherConstants.RollerGains.kD;
      topConfig.Slot0.kS = LauncherConstants.RollerGains.kS;
      topConfig.Slot0.kV = LauncherConstants.RollerGains.kV;
      topConfig.Slot0.kA = LauncherConstants.RollerGains.kA;
      topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      bottomMotor.set(0);
      topMotor.getConfigurator().apply(bottomConfig);
      m_topVoltage.withSlot(0);

      this.start();
  }


  public void stopLauncher(){
    this.goalSpeedRPS = 0;
  } 


  @Override
  public void periodic() {
      if(goalSpeedRPS == 0){
        bottomMotor.setVoltage(0);
        topMotor.setVoltage(0);
      } else {
        m_BottomVoltage.withVelocity(goalSpeedRPS);
        bottomMotor.setControl(m_BottomVoltage);
        m_topVoltage.withVelocity(goalSpeedRPS);
        topMotor.setControl(m_topVoltage);
      }
        SmartDashboard.putNumber("Shooter Speed", bottomMotor.getVelocity().getValueAsDouble()) ; 
        SmartDashboard.putNumber("shooter goal", goalSpeedRPS)    ;
  }

  public void start() {
    this.goalSpeedRPS = 50;
  }

  public void incrementSpeed(double rpsChange) {
    this.goalSpeedRPS += rpsChange;

  }

  public Command incrementSpeedCommand(double rpsChange){
    Command result = runOnce(()-> incrementSpeed(rpsChange));
    return result;
  } 

  public void setSpeed(double rps) {
    this.goalSpeedRPS = rps;
  }

  public Command setSpeedCommand(double rps){
    Command result = runOnce(()-> setSpeed(rps));
    return result;
  } 

  public Command startCommand(){
    Command result = run(this::start);
    return result;
  }

  public Command stopCommand(){
    Command result = run(this::stopLauncher);
    return result;
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
                        bottomMotor.setVoltage(volts.in(Volts));
                        topMotor.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // Record a frame for the wheel motor. 
                        log.motor("bottom")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                bottomMotor.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_rotation.mut_replace(bottomMotor.getPosition().getValueAsDouble(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(bottomMotor.getVelocity().getValueAsDouble(), RadiansPerSecond));
                        log.motor("top")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                topMotor.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_rotation.mut_replace(topMotor.getPosition().getValueAsDouble(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(topMotor.getVelocity().getValueAsDouble(), RadiansPerSecond));

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
