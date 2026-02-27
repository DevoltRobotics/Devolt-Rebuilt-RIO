package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Constants {

    public static final class PoseinField{ //poses para autoacomodación
        public static final Pose2d frontofapriltag = new Pose2d(7.0, 4.0, new Rotation2d(0));
        public static final Pose2d humanPlayer = new Pose2d(0, 0, new Rotation2d(0));
    }

    public static final class TurretsPos{
        public static final Translation2d LeftTurretOffset = new Translation2d(0.165, 0.152); 
        public static final Translation2d RightTurretOffset = new Translation2d(0.165, -0.152); 

    }

    public static final class CANId {
        public static final class CAN_Swerve{
            public static final int PigeonCan = 2;

            public static final int FLEncoderCan = 3;
            public static final int FREncoderCan = 4;
            public static final int RLEncoderCan = 5;
            public static final int RREncoderCan = 6;

            public static final int FLTurnCan = 7;
            public static final int FLDriveCan = 8;
            public static final int FRTurnCan = 9;
            public static final int FRDriveCan = 10;
            public static final int RLTurnCan = 11;
            public static final int RLDriveCan = 12;
            public static final int RRTurnCan = 13;
            public static final int RRDriveCan = 14;
        }
        public static final class CAN_Intake{

            public static final int PivotCan = 15;
            public static final int TransferCan = 16;
            public static final int IntakeCan = 17;
        } 

        public static final class CAN_Shooter{
            public static final int LTurretCan = 18;
            public static final int RFlywheelCan = 19;
            public static final int KickerCan = 20;
            public static final int RTurretCan = 21;
            public static final int LFlywheelCan = 22;
        }
    }

    public static final class LimelightConstants{
        public static final String backLimelightName = "limelight-backll";
        public static final String frontLimelightName = "limelight-frontll"; 
    }

}
