# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Effect of P, I, and D component in an PID controller
* The P-component defines the proportionality factor between cross track error (CTE) and control signal,
 i.e., it accounts for the present error. In this project, the P-term alone would simply increase the steering
 angle proportional to the CTE. Using only the P-component would result in oscillating motion of the car around
 the desired trajectory.  

* The differential component damps the control signal produced by the P-component based
 on the current time derivative of the error. In a sense the D-component accounts for the
trend of the error and thus is able to prevent oscillations. In the absence of constant
external forces or biases in the steering mechanism, a controller comprised of P- and D-
component alone would be sufficient to control the car.

* The I component accounts for accumulated, i.e., integrated CTE and thus can compensate
for biases or external forces acting on the car.

## Parameter Tuning
To come up with a sensible initial estimate for the parameters Kp, Ki, and Kd of the P,I, and D components, respectively
I did a simple back of the envelope calculation: The outer tires of the car leave
the road when the CTE is approximately 3.5. The range of accepted steering angles is -1 to 1.
Based on the assumption that an steering angle of 0.5 would be a good value recover
the car from the edge of the road, the initial value for Kp was set to 0.14. When manually
controlling the car, I did not notice strong crosswinds or steering biases. Hence I set Ki to the very small value of 0.001. Then, I set Kd to 1 and started the
TWIDDLE algorithm described in the class. Starting with a small time-interval over which the error is measured, I gradually increased the interval as the controller got better, until I was able to complete a whole lap.
At this point I let the TWIDDLE algorithm run for a few more iterations before I settled for the following final parameter setting:

Kp = 0.102614, Ki= 0.000389051 Kd = 1.25284
