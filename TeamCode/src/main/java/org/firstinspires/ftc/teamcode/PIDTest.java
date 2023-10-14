import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@config
@teleop
public class PIDTest extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f=0;

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx arm_motor;

    @override
    public void init() {
        controller = new PIDController(p,i,d);
        Telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm_motor = hardwareMap.get(DcmotorEx.class, “arm_motor0”);

        Untitled document	public void loop()
        controller.setPID(p,i,d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPo,s target);
        double  ff - Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData(“pos “. armPos);
        telemetry.addData(“target “, target);
        telemetry.update();
    }
}
