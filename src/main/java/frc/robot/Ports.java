package frc.robot;

public class Ports {
    /*** SUBSYSTEM IDS ***/

    // Controllers
    public static class Joysticks {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    public static class Slapdown {
        public static final int PivotMotor = 1;
        public static final int PivotEncoder = 2;

    }

    public static class EndEffector {
        public static final int IntakeMotor = 3;
        public static final int CenteringMotor = 4;
        public static final int leftBeam = 0;
        public static final int rightBeam = 0;
    }

}