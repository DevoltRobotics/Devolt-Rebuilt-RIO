package frc.robot.Commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ShooterController;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.ShooterController.ShooterResult;

public class SOTFCommand extends Command {

  CommandSwerveDrivetrain drivetrain;

  ShooterSubsystem shooter;
  TurretSubsystem turret;

  public static final double Goal_X_Red = 11.920;
  public static final double Goal_Y_Red = 4.035;

  public static final double Goal_X_Blue = 4.6;
  public static final double Goal_Y_Blue = 4.035;

  double Goal_X = 11.920;
  double Goal_Y = 4.035;

  double yOffstet = 0;
  double xOffset = 0;


  private boolean getAlliance = false;

  private double lastTime = Timer.getFPGATimestamp();
  private double avgDt = 0.02;

  public SOTFCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      double offsety,
      double offsetx) {
    this.drivetrain = drivetrain;
    this.shooter = shooterSubsystem;
    this.turret = turretSubsystem;
    addRequirements(shooter, turret);
    yOffstet = offsety;
    xOffset = offsetx;
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    // filtro para que no brinque
    avgDt = 0.9 * avgDt + 0.1 * dt;

    SmartDashboard.putNumber(turret.getName() + "/SOTF/dt", dt);
    SmartDashboard.putNumber(turret.getName() + "/SOTF/avgDt", avgDt);
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && !getAlliance) {
      if (alliance.get() == Alliance.Red) {
        Goal_X = Goal_X_Red + xOffset;
        Goal_Y = Goal_Y_Red - yOffstet;
        getAlliance = true;
      } else if (alliance.get() == Alliance.Blue) {
        Goal_X = Goal_X_Blue - xOffset;
        Goal_Y = Goal_Y_Blue + yOffstet;
        getAlliance = true;
      } else {
        Goal_X = Goal_X_Red;
        Goal_Y = Goal_Y_Red;
      }
    }
    Pose2d pose = drivetrain.getState().Pose;
    ChassisSpeeds speeds = drivetrain.getState().Speeds;

    Translation2d rotatedTurretOffset = turret.turretOffset.rotateBy(pose.getRotation());
    Translation2d turretFieldPos = pose.getTranslation().plus(rotatedTurretOffset);

    Logger.recordOutput(turret.getName() + "/SOTF/HubPos", new Pose2d(new Translation2d(Goal_X, Goal_Y), new Rotation2d(0)));
    Logger.recordOutput(turret.getName() + "/SOTF/TurretFieldPos", new Pose2d(turretFieldPos, pose.getRotation().plus(turret.getAngle())));


    ShooterResult result = ShooterController.calculate(
        turretFieldPos,
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
        new Translation2d(Goal_X, Goal_Y),
        avgDt
    );

    
    Logger.recordOutput(turret.getName() + "/SOTF/DistanceToGoal", result.distanceToGoal());

    shooter.setVelocity(result.requiredRps());
    turret.setAngle(result.turretFieldAngle().minus(pose.getRotation()).getDegrees());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
