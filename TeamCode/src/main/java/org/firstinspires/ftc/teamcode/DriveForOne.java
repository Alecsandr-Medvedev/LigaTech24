package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DRIVERS_ONE", group = "Pushbot")
// @Disabled
public class DriveForOne extends Drive {

    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("angles = ", robot.angles);
        telemetry.update();
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();


        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        while (!isStarted()) {
            telemetry.update();
        }

        if (opModeIsActive()) {
            robot.move(0, 0);
            while (opModeIsActive()) {
                double Y = gamepad1.left_stick_y;
                double X = -gamepad1.left_stick_x;
                boolean kofLeft = gamepad1.left_bumper;
                boolean kofRight = gamepad1.right_bumper;
                boolean qwerty = gamepad1.a;
                double leftTrigger = -gamepad1.left_trigger;
                double rightTrigger = gamepad1.right_trigger;
                boolean holder = gamepad1.x;
                boolean manipulatorAutoUp = gamepad1.dpad_up;
                boolean manipulatorAutoDown = gamepad1.dpad_down;
                boolean clawOpen = gamepad1.dpad_left;
                boolean clawClose = gamepad1.dpad_right;
                double upperUpDown = gamepad1.right_stick_y;
                boolean reInitEncoder = gamepad1.y;
                gyroDrive(Y, X, kofLeft, kofRight, qwerty, leftTrigger, rightTrigger, holder,
                        manipulatorAutoUp, manipulatorAutoDown, clawOpen, clawClose, upperUpDown, reInitEncoder, robot);
            }
        }
    }

}