## TeamCode Module

Welcome!

Team 524, The Boss Bot's repository

## Contains:

* MecanumOpMode
An abstract class which contains the code for holonomic drive (Mecanum and Omniwheel). Currently
has only the joystick controls. Will be adding more codes for driving at certain angles.

## Structure of robot

```
  Omni-wheel               Mecanum Wheel
   [Front]                    [Front]
motor4  motor3             motor4  motor3
   /-----\                   []-----[]
    |   |                     |     |
    |   |                     |     |
   \-----/                   []-----[]
motor1  motor2             motor1  motor2
```

## Joystick Coordinate/ Position (Hardware Configuration)

```
Starting Position

   [Y-Axis]
     -1
      |
-1 -- o -- 1 [X-axis]
      |
      1

Example Position

   [Y-Axis]
     -1
      | o (0.74, -0.74)
-1 --   -- 1 [X-axis]
      |
      1
```

# API
* `public DcMotor motor1, motor2, motor3, motor4`

 Variables | Explanation
 --- | ---
 `DcMotor motor1` | Bottom left motor (refer to diagram above).
 `DcMotor motor2` | Bottom right motor (refer to diagram above).
 `DcMotor motor3` | Top right motor (refer to diagram above).
 `DcMotor motor4` | Top left motor (refer to diagram above).

* `public void driveOneJoystick(int gamepad, String side, DcMotor motor4, DcMotor motor3, DcMotor motor2, DcMotor motor1)`
 * **Function** Drives the motor according to gamepad1 left or right stick position.
 * **Parameters:**

 Parameters | Explanation
 --- | ---
 `int gamepad` | which gamepad to use: 1 or 2. Default: 1.
 `String side` | Which joystick to use on gamepad: right or left. Default: right.


* `public double getJoystickAngle(double x, double y)`
 * **Function** Gets the angle (in radians) which the joystick is at currently relative to the center/resting point. Refer to joystick coordinates above.
 * **Parameters:**

 Parameters | Explanation
  --- | ---
  double x | x coordinate of gamepad
  double y | y coordinate of gamepad

* `public double getDistance(double x, double y)`
 * **Function** Gets the joystick's distance relative from the center/resting point. Refer to joystick coordinates above.
 * **Parameters:**

 Parameters | Explanation
  --- | ---
  double x | x coordinate of gamepad
  double y | y coordinate of gamepad


  # PID

  ##How PID (actually, PD in this case) works

  User sets setpoints.
  Current values are measured from sensors.
  Error = setpoint - current.
  Output for the motors = Kp*Error + Kd*derivative of Error

  ##Working with sensors
  #####(the hardest part (because Moto G3's do not have gyroscopes) )

  The phone has its own set of x, y and z coordinates and it does not care about the real world. But we do,
  so the phone must care about it too.

  !!!GRAVITY TO THE RESCUE!!!

  The acceleration due to gravity always points downwards. So when the robot is not moving (not accelerating, to be precise),
  the accelerometer is going to measure gravitational acceleration. So,

    1. Wait for 2 seconds for the sensors to stabilize.

    2. Measure the acceleration and store it in the variable g that should not be changed after this.

  Now, we take the unit vector in the direction of `g`. We call it `gprime`. This is the unit vector in the +z direction (DOWNWARD).
  IMPORTANT: `gprime` is in phone's coordinate system (in terms of the phone's x-, y- and z-coordinates).

  Next, we need to get one of the x- and y-axes, and then we can get the other one by using the vector cross product.
  So we choose to get the y-axis. But how do we choose which way to make it point?

  !!!MAGNETIC FIELD TO THE RESCUE!!!

  We find the magnetic field vector in the phone's coordinate system and call it `b`. Now, it is not always perpendicular
  to the gravitational force. So we find the component of `b` perpendicular to `gprime` and call it `bprime`. We then find the
  unit vector in the direction of `bprime` and call it `y`. This is our y-axis. Except that it is NOT our y-axis, because our
  gamefield will be randomly oriented with respect to the earth's magnetic field, and we are entering setpoints
  with respect to the gamefield. So we measure the offset of our gamefield with respect to the earth's magnetic field
  in radians (using a run-off-the-mill compass app), and store it in a variable called `calibration`. Then we rotate
  `y` by `calibration` radians about `g`.

  Finally, we find the x-axis by finding the cross product `y` **×** `g` (remember that in a right-handed coordinate system, like the
  one we want to use, j **×** k = i).

  If you've been following along, you may ask one question. Why bother so much about getting unit vectors, and making
  sure that they are perpendicular? We can make this matrix using those values:

   ```
   [x1 y1 g1]

   [x2 y2 g2]

   [x3 y3 g3]
   ```

