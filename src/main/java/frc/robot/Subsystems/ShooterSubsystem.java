// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX Lflywheel;

  private final TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();

  private VelocityVoltage Velocity = new VelocityVoltage(0);
  

  Pose2d positionInRobot = new Pose2d();
  ChassisSpeeds Speed = new ChassisSpeeds();

  private double desiredVelocity;

  double TurretOut = 0;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(int canId, Pose2d positionInRobot) {
    Lflywheel = new TalonFX(canId, CANBus.roboRIO());

    FlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimit = 35;
    FlywheelConfig.Slot0.kP = 0.15;
    FlywheelConfig.Slot0.kV = 0.115;
    FlywheelConfig.Slot0.kS = 0.4;
    Lflywheel.getConfigurator().apply(FlywheelConfig);

    this.positionInRobot = positionInRobot;
    

    setName("Turret" + canId);
  }

  @Override
  public void periodic() {
    Lflywheel.setControl(Velocity.withVelocity(desiredVelocity));
    SmartDashboard.putNumber("Shooter/Target vel", desiredVelocity);

  }

  public void setVelocity(double velocity) {
    desiredVelocity = velocity;
  }

  public Command SetVelocityCMD(double Vel) {
    return new InstantCommand(() -> {
      setVelocity(Vel);
    });
  }

}
