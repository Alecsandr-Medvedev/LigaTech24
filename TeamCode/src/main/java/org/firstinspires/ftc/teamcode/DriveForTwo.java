package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DRIVERS_TWO", group = "Pushbot")

public class DriveForTwo extends Drive {

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
                boolean holder = gamepad2.x;
                boolean manipulatorAutoUp = gamepad2.dpad_up;
                boolean manipulatorAutoDown = gamepad2.dpad_down;
                boolean clawOpen = gamepad2.dpad_left;
                boolean clawClose = gamepad2.dpad_right;
                double upperUpDown = gamepad2.right_stick_y;
                boolean reInitEncoder = gamepad2.a;
                gyroDrive(Y, X, kofLeft, kofRight, qwerty, leftTrigger, rightTrigger, holder,
                        manipulatorAutoUp, manipulatorAutoDown, clawOpen, clawClose, upperUpDown, reInitEncoder, robot);
            }
        }
    }

}