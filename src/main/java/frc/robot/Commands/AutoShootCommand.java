package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class AutoShootCommand extends ParallelCommandGroup {

    private final TransferSubsystem transferSubsystem;

    public AutoShootCommand(
        CommandSwerveDrivetrain drivetrain,
        ShooterSubsystem leftShooterSubsystem,
        TurretSubsystem leftTurretSubsystem,
        ShooterSubsystem rightShooterSubsystem,
        TurretSubsystem rightTurretSubsystem,
        TransferSubsystem transferSubsystem,
        double xOffset,
        double yOffset
    ) {
        this.transferSubsystem = transferSubsystem;

        SOTFCommand leftSOTF =
            new SOTFCommand(drivetrain, leftShooterSubsystem, leftTurretSubsystem, yOffset, xOffset);

        SOTFCommand rightSOTF =
            new SOTFCommand(drivetrain, rightShooterSubsystem, rightTurretSubsystem, yOffset, xOffset);

      FunctionalCommand transferLogic = new FunctionalCommand(

    // initialize
    () -> {},

    // execute
    () -> {
        boolean leftReady = leftTurretSubsystem.IsReadyToShoot() && leftShooterSubsystem.IsReadyToShoot();
        boolean rightReady = rightTurretSubsystem.IsReadyToShoot() && rightShooterSubsystem.IsReadyToShoot();
        boolean bothReady = leftReady && rightReady;

        SmartDashboard.putBoolean("AutoShoot/LeftReady", leftReady);
        SmartDashboard.putBoolean("AutoShoot/RightReady", rightReady);
        SmartDashboard.putBoolean("AutoShoot/BothReady", bothReady);

        if (bothReady) {
            transferSubsystem.setSpeeds(1, 120);
            SmartDashboard.putBoolean("AutoShoot/TransferOn", true);
        } else {
            transferSubsystem.setSpeeds(0, 0);
            SmartDashboard.putBoolean("AutoShoot/TransferOn", false);
        }
    },

    (interrupted) -> {
        transferSubsystem.setSpeeds(0, 0);
        SmartDashboard.putBoolean("AutoShoot/TransferOn", false);
    },

    // isFinished
    () -> false,

    transferSubsystem
);

        addCommands(
            leftSOTF,
            rightSOTF,
            transferLogic
        );
    }


}