// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

  private SparkMax Lturret;
  private SparkMaxConfig TurretConfig = new SparkMaxConfig();

  public final Translation2d turretOffset;

  PIDController turretPidController = new PIDController(.01, 0, 0);

  private double desiredAngle = 0;
  double TurretOut = 0;

  double turretOffsetEnc = 0;

  double turretRelativePos = 0;
  double normalizedAngle = 0;

  double Upperlimit = 270;
  double Lowerlimit = -90;

  // ---------------- Mechanism2d ----------------
  private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d root = mech.getRoot("TurretRoot", 1.5, 1.5);

  // Azul = posición real
  private final LoggedMechanismLigament2d turretLigament = root.append(new LoggedMechanismLigament2d(
      "TurretActual",
      1.0,
      0,
      6,
      new Color8Bit(Color.kBlue)));

  // Rojo = setpoint (deseado)
  private final LoggedMechanismLigament2d targetLigament = root.append(new LoggedMechanismLigament2d(
      "TurretTarget",
      1.2,
      0,
      4,
      new Color8Bit(Color.kRed)));

  // Verde / Naranja = límites (visuales)
  private final LoggedMechanismLigament2d upperLimitLigament = root.append(new LoggedMechanismLigament2d(
      "UpperLimit",
      1.35,
      90,
      2,
      new Color8Bit(Color.kGreen)));

  private final LoggedMechanismLigament2d lowerLimitLigament = root.append(new LoggedMechanismLigament2d(
      "LowerLimit",
      1.35,
      -270,
      2,
      new Color8Bit(Color.kOrange)));
  // ------------------------------------------------

  public TurretSubsystem(int canId, Translation2d turretOffset) {
    Lturret = new SparkMax(canId, MotorType.kBrushless);
    this.turretOffset = turretOffset;

    TurretConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    TurretConfig.encoder
        .positionConversionFactor(7.2);

    Lturret.configure(TurretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setName("TurretSubsystem" + canId);

    Logger.recordOutput(getName() + "/TurretMechanism2d", mech);

    resetOffsetCMD().schedule();
  }

  @Override
  public void periodic() {
    turretRelativePos = Lturret.getEncoder().getPosition() - turretOffsetEnc;

    if (desiredAngle > Upperlimit) {
      normalizedAngle = desiredAngle - 360;
    } else if (desiredAngle < Lowerlimit) {
      normalizedAngle = desiredAngle + 360;
    } else {
      normalizedAngle = desiredAngle;
    }

    turretPidController.setSetpoint(normalizedAngle);

    double pidOut = turretPidController.calculate(turretRelativePos);
    TurretOut = MathUtil.clamp(pidOut, -0.5, 0.5);

    if (turretRelativePos >= Upperlimit && TurretOut > 0) {
      TurretOut = 0;
    }
    if (turretRelativePos <= Lowerlimit && TurretOut < 0) {
      TurretOut = 0;
    }

    Lturret.set(TurretOut);

    turretLigament.setAngle(turretRelativePos);
    targetLigament.setAngle(normalizedAngle);
    upperLimitLigament.setAngle(Upperlimit);
    lowerLimitLigament.setAngle(Lowerlimit);

    SmartDashboard.putNumber("Turret/RelPos", turretRelativePos);
    SmartDashboard.putNumber("Turret/Desired", desiredAngle);
    SmartDashboard.putNumber("Turret/Norm", normalizedAngle);
    SmartDashboard.putNumber("Turret/Out", TurretOut);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(turretRelativePos);
  }

  public void setAngle(double angle) {
    desiredAngle = angle;
  }

  public void resetOffset() {
    turretOffsetEnc = Lturret.getEncoder().getPosition();
  }

  public Command resetOffsetCMD() {
    return new InstantCommand(() -> {
      resetOffset();
    });
  }

  public Command SetTurretPosCMD(double Pos) {
    return new InstantCommand(() -> {
      setAngle(Pos);
    }, this);
  }
}