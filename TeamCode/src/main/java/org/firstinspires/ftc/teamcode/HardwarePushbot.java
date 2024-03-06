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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This is NOT an opmode.
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class HardwarePushbot {
    /* Public OpMode members. */
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;

    public DcMotor/*Ex*/ upper = null;

    public Servo claw = null;
    public Servo holderRight = null;
    public Servo holderLeft = null;
    public DistanceSensor sensorRange;

    public BNO055IMU imu = null;
    public Orientation angles;
    public VuforiaTrackables targets   = null;
    public VuforiaTrackables targets_new   = null;
    public String str = null;
    public VectorF vect = null;
    public Orientation orint = null;
     public static final double initClawCof = 0.6;
    public double clawCloseCof = 0.4;
    public double clawOpenCof = 0.45;
    public int epsilont = 5;
    public int UP_POSITION = -740;
    public int DOWN_POSITION = 0;
    public int HALF_POSITION = (UP_POSITION - DOWN_POSITION) / 2;
    public double minSpeedUpper = 0.2;
    public double maxSpeedUpper = 1;
    public double holderStartPosition = 0;
    public double holderEndPosition = 0.7;

    public double upperAutoSpeed = 0.2;

    public double speedUpper;
    public int newTarget = DOWN_POSITION;

    public int speedAddTarget = 15;

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
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        rightBack = hwMap.get(DcMotor.class, "rightBack");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        upper = /*(DcMotorEx)*/ hwMap.get(DcMotor.class, "upper");
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");


        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        upper.setDirection(DcMotorSimple.Direction.FORWARD);
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
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

        claw = hwMap.get(Servo.class, "claw");
        holderLeft = hwMap.get(Servo.class, "holderLeft");
        holderRight = hwMap.get(Servo.class, "holderRight");

        claw.setPosition(initClawCof);
        holderRight.setPosition(180);
        holderLeft.setPosition(0);


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

    public void setTargetMotor(double speed,
                               int target, DcMotor motor) {

        motor.setTargetPosition(target);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(speed));

    }

    public void updateEncoder(DcMotor motor){
        if (!motor.isBusy()){
            if (Math.abs(newTarget - motor.getCurrentPosition()) > epsilont){
                setTargetMotor(0.2, newTarget, motor);
            }
            else{
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void upHolder(){
        holderLeft.setPosition(holderStartPosition);
        holderRight.setPosition(1 - holderStartPosition);
    }
    public void downHolder(){
        holderLeft.setPosition(holderEndPosition);
        holderRight.setPosition(1 - holderEndPosition);
    }

    public void closeClaw(){
        claw.setPosition(clawCloseCof);
    }
    public void openClaw(){
        claw.setPosition(clawOpenCof);
    }

    public void setNewDownPosition(){
        DOWN_POSITION = upper.getCurrentPosition();
        HALF_POSITION = (UP_POSITION - DOWN_POSITION) / 2;
    }

    public void setNewUpPosition(){
        UP_POSITION = upper.getCurrentPosition();
        HALF_POSITION = (UP_POSITION - DOWN_POSITION) / 2;
    }

    public void upperAutoUp(){
        setTargetMotor(upperAutoSpeed, UP_POSITION, upper);
    }
    public void upperAutoDown(){
        setTargetMotor(upperAutoSpeed, DOWN_POSITION, upper);
    }

    public void moveUpper(double speedCof){
        speedUpper = maxSpeedUpper * Math.pow((1 - Math.abs(HALF_POSITION
                - upper.getCurrentPosition()) / (HALF_POSITION * -1.0)), 1);
        if (speedUpper < minSpeedUpper){
            speedUpper = minSpeedUpper;
        }
        newTarget += speedAddTarget * speedCof;

        setTargetMotor(speedUpper, newTarget, upper);
    }
    public void stopUpper(){
        newTarget = upper.getCurrentPosition();
        setTargetMotor(upperAutoSpeed * 2, newTarget, upper);
    }
}

