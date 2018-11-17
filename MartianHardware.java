/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
public class MartianHardware
{
    /* Public OpMode members. */
    public DcMotor  rearLeftDrive   = null;
    public DcMotor  rearRightDrive  = null;
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    
    public DcMotor  motorArmLeft     = null;
    public DcMotor  motorArmRight     = null;
    
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo    contLinear  = null;
    
    public Servo    sensorArm   = null;
    
    public DistanceSensor sensorRange = null;
    public ColorSensor sensorColor = null;

    public static final double MID_SERVO       =  0.45 ;
    public static final double ARM_UP_POWER    =  0.1 ;
    public static final double ARM_DOWN_POWER  = -0.1;
   


    /* Vuforia Variables */
    public static final String TAG = "Vuforia VuMark Martians";
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    public VuforiaLocalizer.Parameters parameters = null;
    public VuforiaTrackables relicTrackables = null;
    public VuforiaTrackable relicTemplate = null;
    public RelicRecoveryVuMark vuMark = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MartianHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        //int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        //parameters = new VuforiaLocalizer.Parameters();
        
        //parameters.vuforiaLicenseKey = "AbCuv8X/////AAAAGf9WW1xLgEveg2n/MGV4F9pniTB4OvNJyhAd4UfqO7GF2/9tmJ+iLqqBSF1eCbql5kL/X+X5kKSEGK+kLQT298wVsRgWxA8o7UEZAIbJf85mI9s4WFpG3ZBYPU/EOGMFL0jYjkzS88kFHHKvT/mNwnJ5odymxzNihE92V2V/lCcspxOpccIVPo7nIoj//66jccfjFUfwcLLzWYGMGRO0PoOdI1lsUw7ALiWrfoyyFu1tM+UYTL1d+fmWd+MDV9gFOk13B/FD68SRxQPedAggg9JZY3h1be7gQsF4ybRRkdBIiM/MFv95blSOd9OKPgv6bf7c5lZiDUTgf25ohh+TdE2oYKTpi4iaZtSUcVvX5Qep";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        //this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        //relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        //relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Define and Initialize Motors
        rearLeftDrive  = hwMap.get(DcMotor.class, "motorRearLeft");
        rearRightDrive = hwMap.get(DcMotor.class, "motorRearRight");
        frontLeftDrive  = hwMap.get(DcMotor.class, "motorFrontLeft");
        frontRightDrive = hwMap.get(DcMotor.class, "motorFrontRight");
        
        motorArmLeft    = hwMap.get(DcMotor.class, "motorArmLeft");
        motorArmRight    = hwMap.get(DcMotor.class, "motorArmRight");
        
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD); 
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); 
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        rearLeftDrive.setPower(0); 
        rearRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize all installed servos.
        leftClaw  = hwMap.get(Servo.class, "servoLeftClaw");
        rightClaw = hwMap.get(Servo.class, "servoRightClaw");
        contLinear = hwMap.get(Servo.class, "servoContLinear");
        
        //sensorArm = hwMap.get(Servo.class, "servoSensorArm");
        
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
        //sensorArm.setPosition(MID_SERVO);
        
        // Define and initialize all Sensors
        //sensorRange = hwMap.get(DistanceSensor.class, "sensorColorRange");
        //sensorColor = hwMap.get(ColorSensor.class, "sensorColorRange");
        
    }
 }
