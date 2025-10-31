package frc.robot;

public class Ports {
    /*** SUBSYSTEM IDS ***/

    // Controllers
    public static class Joysticks {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    public static class Pivot {
        public static final int PivotMotor = 11;
        public static final int PivotEncoder = 12;
        public static final int TopLimitSwitch = 0;
        public static final int BotLimitSwitch = 10;
    }

    public static class EndEffector {
        public static final int IntakeMotor = 13;
        public static final int CenteringMotor = 14;
        public static final int leftDIO = 2;
        public static final int rightDIO = 3;
    }

    public static class Climb {
        public static final int Motor = 9;
        public static final int topBeamBreak = 5;
        public static final int botBeamBreak = 6;
        public static final double speed = 0.5;
    }

}