// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANId.CAN_Intake;
import frc.robot.Constants.CANId.CAN_Shooter;


public class TransferSubsystem extends SubsystemBase {
  private DigitalInput laserLeft = new DigitalInput(0);
  private DigitalInput laserRight = new DigitalInput(1);

  private SparkMax beltMotor = new SparkMax(CAN_Intake.TransferCan, MotorType.kBrushless);

  private TalonFX kickerMotor = new TalonFX(CAN_Shooter.KickerCan, CANBus.roboRIO());
  private DutyCycleOut Power = new DutyCycleOut(0);

  private final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
  private final SparkMaxConfig beltConfig = new SparkMaxConfig();

  public TransferSubsystem() {
    beltConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);

    beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.SupplyCurrentLimit = 35;

    kickerMotor.getConfigurator().apply(kickerConfig);

  }

  public void setSpeeds(double beltSpeed, double kickerSpeed) {
    beltMotor.set(beltSpeed);
    kickerMotor.setControl(Power.withOutput(kickerSpeed));
  }

  public Command TransferShootCMD(TransferSubsystem transferSubsystem) {
    return new InstantCommand(() -> {
      transferSubsystem.setSpeeds(1, 1);
    });
  }

  public Command StopTransferCMD(TransferSubsystem transferSubsystem) {
    return new InstantCommand(() -> {
      transferSubsystem.setSpeeds(0, 0);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("leftTransfer", laserLeft.get());
    SmartDashboard.putBoolean("rightTransfer", laserRight.get());
  }
}
