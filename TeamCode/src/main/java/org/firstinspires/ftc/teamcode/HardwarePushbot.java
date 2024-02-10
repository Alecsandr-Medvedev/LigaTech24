/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;

    public DcMotor manipulator = null;
    public DcMotor upper = null;

    public Servo claw = null;
    public Servo turn = null;
    public Servo updown = null;

    public double position = 0.6;

    public DigitalChannel btnManipulator = null;

    public BNO055IMU imu = null;
    public Orientation angles;
    public Acceleration gravity;

    public WebcamName webcamName = null;
    public OpenGLMatrix lastLocation   = null;
    public VuforiaLocalizer vuforia    = null;
    public VuforiaTrackables targets   = null;
    public VuforiaTrackables targets_new   = null;
    public boolean targetVisible       = false;

    public String str = null;
    public VectorF vect = null;
    public Orientation orint = null;

    private static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;

    public static final String VUFORIA_KEY =
            "AVBuwuj/////AAABme3qqHtZakBEoej2qn+K61cejFDMxTrewaZmF7T0aSbPZoYBxR7OnV8UwQvv4JDD566lMwT8UeL1sgLPkkL//OTN6cSnIm5x01bCTQFQ7NX4KxvMezRPXCltLug3QzE5J9JoyCPWevWrkkSP+9ZTTI77Naab43kdLmtNF47fDThZVvp1W8t2LsGSUOdxOv7dSMrVsvDhhJZLsIJtvRFFaiiiWD5QOWlISG7780qEPpYSSfAlel39pxm4/A+wcK2Siwp9eVPY70TSUKuN/eHZ7a5ihZDJKKSsOQ2CtlzwoQKvKh79wNi/P+lY++oeFgobuHXS7pPXPjldLrtTMXAsgfXlcHcW73XSloA9EdgwFyMZ";

    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public int v = 0;
    public static final double k = 12 * 2;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;


    /* Constructor */
    public HardwarePushbot(){

    }

    public void move(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    public void move(double powerR, double powerL) {
        move(powerR, powerL, powerL, powerR);
    }


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
//        webcamName = hwMap.get(WebcamName.class, "Webcam 1");
//        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//
//        parameters.cameraName = webcamName;
//        parameters.useExtendedTracking = false;
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        System.out.println(vuforia);
//
//        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
//        targets_new = this.vuforia.loadTrackablesFromAsset("FTC_VUF");
//
//        allTrackables.addAll(targets);
//        allTrackables.addAll(targets_new);

//        identifyTarget_new(0, "test_1",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);
//        identifyTarget_new(1, "test_2",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);
//        identifyTarget_new(2, "Cube", halfTile,  halfField,2 * mmPerInch, 0, 0 , 0);
//
//        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
//        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
//        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
//        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);
//
//
//        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
//        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
//        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

//        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));
//
//
//
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
//        }
//
//        targets.activate();
//        targets_new.activate();


        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        rightBack = hwMap.get(DcMotor.class, "rightBack");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        manipulator = hwMap.get(DcMotor.class, "manip");
        upper = hwMap.get(DcMotor.class, "upper");


        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        manipulator.setDirection(DcMotorSimple.Direction.REVERSE);
        upper.setDirection(DcMotorSimple.Direction.FORWARD);
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        manipulator.setPower(0);
        upper.setPower(0);


        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        manipulator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hwMap.get(Servo.class, "claw");
        turn = hwMap.get(Servo.class, "turn");
        updown = hwMap.get(Servo.class, "updown");
        claw.setPosition(0.6);
        turn.setPosition(0);
        updown.setPosition(position);

    }

    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    void    identifyTarget_new(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets_new.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public int getPositionRB(){
        return rightBack.getCurrentPosition();
    }

    public int getPositionRF(){
        return rightFront.getCurrentPosition();
    }
    public int getPositionLB(){
        return leftBack.getCurrentPosition();
    }
    public int getPositionLF(){
        return leftFront.getCurrentPosition();
    }

}

