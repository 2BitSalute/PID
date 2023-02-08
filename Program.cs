using System;
using System.Collections.Generic;

namespace Pid
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");

            var p = 1.0; // weight current errors more
            var i = 1;
            var d = 0.01; // 0.0 would ignore future potential errors
            var l = 50; // number of iterations

            var feedback = 0.0;

            var feedbackList = new List<double>();
            var timeList = new List<int>();
            var setPointList = new List<double>();

            var pid = new PidController(p, i, d);
            // pid.SetPoint = 0.0;
            pid.SetPoint = 1;
            pid.SampleTime = new TimeSpan(days: 0, hours: 0, minutes: 0, seconds: 0, milliseconds: 10); // 0.01 seconds

            for (int index = 1; index <= l; index++)
            {
                pid.Update(feedback);
                var output = pid.Output;

                if (pid.SetPoint > 0)
                {
                    feedback += (output - (1.0 / index));
                }

                if (index > 9)
                {
                    pid.SetPoint = 1;
                }

                System.Threading.Thread.Sleep(pid.SampleTime * 2);

                feedbackList.Add(feedback);
                setPointList.Add(pid.SetPoint);
                timeList.Add(index);

                Console.WriteLine($"Feedback: {feedback}");
                Console.WriteLine($"SetPoint: {pid.SetPoint}");
                Console.WriteLine($"Index   : {index}");
                Console.WriteLine($"Output  : {output}\n");
            }

            // time_sm = np.array(time_list)
            // time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
            // feedback_smooth = spline(time_list, feedback_list, time_smooth)
        }
    }

    // Proportional-Integral-Derivative
    public class PidController
    {
        DateTime currentTime;
        DateTime lastTime;

        double pTerm;
        double iTerm;
        double dTerm;
        double lastError;

        public PidController(double p = 0.2, double i = 0.0, double d = 0.0, DateTime currentTime = default)
        {
            this.Kp = p;
            this.Ki = i;
            this.Kd = d;

            this.SampleTime = new TimeSpan(0);

            this.SetCurrentTime(currentTime);
            this.lastTime = this.currentTime;

            this.Clear();
        }

        public void SetCurrentTime(DateTime currentTime = default)
        {
            if (currentTime == default(DateTime))
            {
                this.currentTime = DateTime.Now;
            }
            else
            {
                this.currentTime = currentTime;
            }
        }

        // Clears the PID computations and coefficients
        public void Clear()
        {
            this.SetPoint = 0.0;

            this.pTerm = 0.0;
            this.iTerm = 0.0;
            this.dTerm = 0.0;
            this.lastError = 0.0;

            // Windup Guards
            this.WindupGuard = 20.0;

            this.Output = 0.0;
        }

        public double SetPoint { get; set; }

        public double Output { get; private set; }

        // Determines how aggressively the PID reacts to the current error with setting Proportional Gain
        public double Kp { get; set; }

        // Determines how aggressively the PID reacts to the current error with setting Integral Gain
        public double Ki { get; set; }

        // Determines how aggressively the PID reacts to the current error with setting Derivative Gain
        public double Kd { get; set; }

        /*
            Integral windup, also known as integrator windup or reset windup,
            refers to the situation in a PID feedback controller where
            a large change in setpoint occurs (say a positive change)
            and the integral terms accumulates a significant error
            during the rise (windup), thus overshooting and continuing
            to increase as this accumulated error is unwound
            (offset by errors in the other direction).
            The specific problem is the excess overshooting. */
        public double WindupGuard { get; set; }

        // PID that should be updated at a regular interval.
        // Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        public TimeSpan SampleTime { get; set; }

        private void ApplyWindupGuard()
        {
            if (this.iTerm < 0 - this.WindupGuard)
            {
                this.iTerm = 0 - this.WindupGuard;
            }
            else if (this.iTerm > this.WindupGuard)
            {
                this.iTerm = this.WindupGuard;
            }
        }

        // Calculates PID value for given reference feedback
        public void Update(double feedbackValue, DateTime currentTime = default)
        {
            // The difference between the desired and the current value
            var error = this.SetPoint - feedbackValue;
            Console.WriteLine($"Error: {error} = {this.SetPoint} - {feedbackValue}");

            this.SetCurrentTime(currentTime);
            var deltaTime = (this.currentTime - this.lastTime).TotalSeconds;
            Console.WriteLine($"Delta time: {deltaTime}");
            var deltaError = error - this.lastError;
            Console.WriteLine($"Delta error: {error} - {this.lastError} = {deltaError}");

            if (deltaTime >= this.SampleTime.TotalSeconds)
            {
                this.pTerm = this.Kp * error;
                Console.WriteLine($"pTerm: {this.pTerm}");
                this.iTerm += error * deltaTime;
                this.ApplyWindupGuard();
                Console.WriteLine($"iTerm: {this.iTerm}");

                this.dTerm = 0.0;
                if (deltaTime > 0)
                {
                    this.dTerm = deltaError / deltaTime;
                }
                Console.WriteLine($"dTerm: {this.dTerm}");

                // Remember the last time and last error for the next calculation
                this.lastTime = this.currentTime;
                this.lastError = error;

                // New value to try!
                this.Output = this.pTerm + (this.Ki * this.iTerm) + (this.Kd * this.dTerm);
            }
        }
    }
}
