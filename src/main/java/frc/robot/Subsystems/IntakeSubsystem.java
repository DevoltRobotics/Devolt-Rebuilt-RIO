// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANId.CAN_Intake;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX rollerMotor = new TalonFX(CAN_Intake.IntakeCan, CANBus.roboRIO());
  private SparkMax pivot = new SparkMax(CAN_Intake.PivotCan, MotorType.kBrushless);

  private double desiredPosition = 0.3;

  private PIDController pivotPID = new PIDController(3, 0, 0);

  private final DoubleSubscriber kPSub;
  private final DoubleSubscriber kISub;
  private final DoubleSubscriber kDSub;

  private double kP = 3;
  private double kI = 0;
  private double kD = 0;

  private DutyCycleOut RollerDutyCycle = new DutyCycleOut(0);

  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();

  private AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();

  private double PivotOut = 0;

  public double intakeUpPos = 0.17;

  public double intakeDownPos = 0.35;

  public double intakeDownSafePos = 0.3;

  double tolerance = 0.004;

  public IntakeSubsystem() {
    pivotConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    pivotConfig.absoluteEncoder
        .positionConversionFactor(1);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 35;

    rollerMotor.getConfigurator().apply(rollerConfig);

    var table = NetworkTableInstance.getDefault().getTable("PivotPID");

    table.getDoubleTopic("kP").publish().set(kP);
    table.getDoubleTopic("kI").publish().set(kI);
    table.getDoubleTopic("kD").publish().set(kD);

    kPSub = table.getDoubleTopic("kP").subscribe(kP);
    kISub = table.getDoubleTopic("kI").subscribe(kI);
    kDSub = table.getDoubleTopic("kD").subscribe(kD);

    pivotPID.setTolerance(tolerance);
  }

  @Override
  public void periodic() {
    pivotPID.setSetpoint(desiredPosition);

    PivotOut = pivotPID.calculate(pivotEncoder.getPosition());

    double newkP = kPSub.get();
    double newkI = kISub.get();
    double newkD = kDSub.get();

    if (newkP != kP || newkI != kI || newkD != kD) {
      kP = newkP;
      kI = newkI;
      kD = newkD;

      pivotPID.setPID(kP, kI, kD);
    }

    if (pivotEncoder.getPosition() > 0.382) {
      pivot.set(0);
    } else {
      pivot.set(PivotOut);
    }
    
    Logger.recordOutput("Pivot/setpoint", desiredPosition);
    Logger.recordOutput("Pivot/error", pivotPID.getError());
    Logger.recordOutput("Pivot/output", PivotOut);
    Logger.recordOutput("Pivot/Pos", pivotEncoder.getPosition());
  }

  public void setRollerSpeed(double desiredRollerSpeed) {
    rollerMotor.setControl(RollerDutyCycle.withOutput(desiredRollerSpeed));

  }

  public void setPosition(double position) {
    desiredPosition = position;
  }

  public Command pivotDownSafeCMD() {
    return new InstantCommand(() -> {
      setPosition(intakeDownSafePos);
    }, this);
  }

  public Command pivotDownCMD() {
    return new InstantCommand(() -> {
      setPosition(intakeDownPos);
    }, this);
  }

  public Command pivotUpCMD() {
    return new InstantCommand(() -> setPosition(intakeUpPos), this);
  }

  public Command IntakeinCMD(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(() -> {
      intakeSubsystem.setRollerSpeed(-0.7);
    });
  }

  public Command IntakeOutCMD(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(() -> {
      intakeSubsystem.setRollerSpeed(0.7);
    });
  }

  public Command IntakeStopCMD(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(() -> {
      intakeSubsystem.setRollerSpeed(0);
    });
  }

}
