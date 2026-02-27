package frc.robot.Commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
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

public class StoreBallsCommand extends Command {

  CommandSwerveDrivetrain drivetrain;

  ShooterSubsystem shooter;
  TurretSubsystem turret;

  double Goal_X_Red = 12.8;

  double Goal_X_Blue = 3.496;

  double GoalRight_Y = 5.633;
  double GoalLeft_Y = 2.644;

  double Goal_X = 0;
  double Goal_Y = 0;

  private boolean getAlliance = false;

  private double lastTime = Timer.getFPGATimestamp();
  private double avgDt = 0.02;

  public StoreBallsCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem) {
    this.drivetrain = drivetrain;
    this.shooter = shooterSubsystem;
    this.turret = turretSubsystem;
    addRequirements(shooter, turret);
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    // filtro para que no brinque
    avgDt = 0.9 * avgDt + 0.1 * dt;

    SmartDashboard.putNumber("SOTF/dt", dt);
    SmartDashboard.putNumber("SOTF/avgDt", avgDt);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Pose2d pose = drivetrain.getState().Pose;


    if (alliance.isPresent() && !getAlliance) {
      if (alliance.get() == Alliance.Red) {
        if (pose.getY() > 4) {
            Goal_X = Goal_X_Red;
            Goal_Y = GoalRight_Y;
        }else if(pose.getY() < 4){
            Goal_X = Goal_X_Red;
            Goal_Y = GoalLeft_Y;
        }  
      } else if (alliance.get() == Alliance.Blue) {
        if (pose.getY() > 4) {
            Goal_X = Goal_X_Blue;
            Goal_Y = GoalRight_Y;
        }else if(pose.getY() < 4){
            Goal_X = Goal_X_Blue;
            Goal_Y = GoalLeft_Y;
        }  
      } else {
        Goal_X = Goal_X_Red;
        Goal_Y = GoalRight_Y;
      }
    }
    ChassisSpeeds speeds = drivetrain.getState().Speeds;
    Translation2d turretFieldPos = pose.getTranslation().plus(turret.turretOffset.rotateBy(pose.getRotation()));

    ShooterResult result = ShooterController.calculate(
        turretFieldPos,
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
        new Translation2d(Goal_X, Goal_Y),
        avgDt
    );

    shooter.setVelocity(result.requiredRps());
    turret.setAngle(result.turretFieldAngle().minus(pose.getRotation()).getDegrees());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
