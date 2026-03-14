package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class AutoShootCommand extends ParallelCommandGroup{
    CommandSwerveDrivetrain drivetrain;
    ShooterSubsystem leftshooterSubsystem;
    TurretSubsystem leftturretSubsystem;
    ShooterSubsystem rightshooterSubsystem;
    TurretSubsystem rightturretSubsystem;
    TransferSubsystem transferSubsystem;
    double xOffset = 0;
    double yOffstet = 0;

    public AutoShootCommand(CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem leftshooterSubsystem,
      TurretSubsystem leftturretSubsystem,
      ShooterSubsystem rightshooterSubsystem,
      TurretSubsystem rightturretSubsystem,
      TransferSubsystem transferSubsystem,
      double xOffset,
      double yOffstet
      ){
        this.leftturretSubsystem = leftturretSubsystem;
        this.leftshooterSubsystem = leftshooterSubsystem;
        this.rightturretSubsystem = rightturretSubsystem;
        this.rightshooterSubsystem = rightshooterSubsystem;
        this.drivetrain = drivetrain;
        this.transferSubsystem = transferSubsystem;
        this.xOffset = xOffset;
        this.yOffstet = yOffstet;

        addCommands(
            new SOTFCommand(drivetrain, leftshooterSubsystem, leftturretSubsystem, yOffstet, xOffset),
            new SOTFCommand(drivetrain, rightshooterSubsystem, rightturretSubsystem, yOffstet, xOffset),
            new RunCommand(()->{
                if (leftturretSubsystem.IsReadyToShoot() && rightturretSubsystem.IsReadyToShoot()) {
                transferSubsystem.setSpeeds(1, 120);
                }else{
                    transferSubsystem.setSpeeds(0,0);
                }
            }, transferSubsystem)

        );

    }


    
}
