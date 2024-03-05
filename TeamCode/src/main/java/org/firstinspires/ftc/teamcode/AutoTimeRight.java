package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwarePushbot.k;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name = "AUTO_RIGHT_TIME", group = "Pushbot")
public class AutoTimeRight extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();

    private ElapsedTime runtime = new ElapsedTime();

    private int a = 0;
    public static int TimeToGo = 1;
    public double X = 0.7;
    public float lastAngel = 0;
    public double Y = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        while (a == 0) {
//            man.runOpMode(robot);
            telemetry.addData("picture", robot.str);
            telemetry.addData("picture", robot.vect);
            telemetry.addData("picture", robot.orint);
            telemetry.addData("v", robot.v);
            telemetry.update();
            if (opModeIsActive()) {
                a = 1;
            }
        }
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        runtime.reset();
        lastAngel = robot.angles.firstAngle;


        while (opModeIsActive() && (runtime.seconds() < TimeToGo)) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            robot.move(X, -X);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

    }
}
