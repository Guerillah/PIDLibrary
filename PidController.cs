//This library is a port of Brett Beauregard's Arduino PID library with some variations:
//http://www.arduino.cc/playground/Code/PIDLibrary

using System;
using System.Threading;

namespace PID
{
    public class PidController
    {
        private readonly Func<float> _readOutput;
        private readonly Func<float> _readProcess;
        private readonly Func<float> _readSetPoint;
        private readonly Action<float> _writeOutput;
        private Timer _computeTimer;
        private ControllerDirection _controllerDirection;
        private ControllerMode _controllerMode;
        private float _derivativeGain;
        private float _integralGain;
        private float _iTerm;
        private float _kd;
        private float _ki;
        private float _kp;
        private float _lastError;
        private float _lastInput;
        private float _proportionalGain;
        private TimeSpan _samplingRate;

        /// <summary>
        ///     Instantiates a new PID controller class. The <see cref="Run" /> method must be called to trigger computation.
        /// </summary>
        /// <param name="samplingRate"> The rate at which the algorithm will compute a new output./// </param>
        /// <param name="outputMinimum">The minimum value the output can be written to. The output influences the process value.</param>
        /// <param name="outputMaximum">The maximum value the output can be written to. The output influences the process value.</param>
        /// <param name="readProcess">
        ///     A function to read the process value. The process value is what you are attempting to
        ///     control.
        /// </param>
        /// <param name="readOutput">A function to read the output. The output influences the process value.</param>
        /// <param name="writeOutput">A function to write to the output. The output influences the process value.</param>
        /// <param name="readSetPoint">A function to read the target set-point for the process.</param>
        /// <param name="proportionalGain">The proportional gain to use.</param>
        /// <param name="integralGain">The integral gain to use.</param>
        /// <param name="derivativeGain">The derivative gain to use.</param>
        /// <param name="controllerDirection">The direction to control the output if you want the process to increase in value..</param>
        /// <param name="controllerMode">The mode to start out the PID controller in.</param>
        /// <exception cref="ArgumentNullException">
        ///     Thrown when <see cref="readProcess" /> or <see cref="readOutput" /> or <see cref="writeOutput" /> or
        ///     <see cref="readSetPoint" /> are null.
        /// </exception>
        public PidController(TimeSpan samplingRate, float outputMinimum, float outputMaximum, Func<float> readProcess,
            Func<float> readOutput,
            Action<float> writeOutput, Func<float> readSetPoint, float proportionalGain, float integralGain,
            float derivativeGain, ControllerDirection controllerDirection, ControllerMode controllerMode)
        {
            if (readProcess == null)
                throw new ArgumentNullException(nameof(readProcess), "Read process must not be null.");
            if (readOutput == null)
                throw new ArgumentNullException(nameof(readOutput), "Read output must not be null.");
            if (readSetPoint == null)
                throw new ArgumentNullException(nameof(readSetPoint), "Read set-point must not be null.");
            if (writeOutput == null)
                throw new ArgumentNullException(nameof(writeOutput), "Write output must not be null.");

            SamplingRate = samplingRate;
            SetOutputLimits(outputMinimum, outputMaximum);
            _readProcess = readProcess;
            _readOutput = readOutput;
            _writeOutput = writeOutput;
            _readSetPoint = readSetPoint;
            ProportionalGain = proportionalGain;
            IntegralGain = integralGain;
            DerivativeGain = derivativeGain;
            ControllerDirection = controllerDirection;
            ControllerMode = controllerMode;
        }

        /// <summary>
        ///     Gets whether the PID controller is currently running/updating.
        /// </summary>
        public bool IsRunning { get; private set; }

        /// <summary>
        ///     Gets the maximum output value set via the <see cref="SetOutputLimits" /> method.
        /// </summary>
        public float OutputMaximum { get; private set; }

        /// <summary>
        ///     Gets the minimum output value set via the <see cref="SetOutputLimits" /> method.
        /// </summary>
        public float OutputMinimum { get; private set; }

        /// <summary>
        ///     Gets or sets the rate at which the PID controller will run its algorithm and compute a new output value. If the PID
        ///     controller is already running, the computation interval will be changed to match this value.
        /// </summary>
        public TimeSpan SamplingRate
        {
            get { return _samplingRate; }
            set
            {
                if (value.TotalMilliseconds.Equals(_samplingRate.TotalMilliseconds))
                    return;

                var newSamplingRate = value;

                if (newSamplingRate.TotalMilliseconds < 0)
                    newSamplingRate = TimeSpan.FromMilliseconds(0);

                var samplingRateRatio = newSamplingRate.TotalMilliseconds/_samplingRate.TotalMilliseconds;

                _samplingRate = value;

                if (_computeTimer != null)
                    _computeTimer.Change(TimeSpan.FromMilliseconds(0), _samplingRate);

                if (double.IsInfinity(samplingRateRatio) || double.IsNaN(samplingRateRatio))
                    return;

                _ki *= (float) samplingRateRatio;
                _kd /= (float) samplingRateRatio;
            }
        }

        /// <summary>
        ///     Gets the current controller dirrection. Sets whether the PID controller will be connected to a (DIRECT) acting
        ///     process whereby increasing the output value increases the
        ///     process value OR a (REVERSE) acting process whereby increasing the output value decreases the process value.
        /// </summary>
        public ControllerDirection ControllerDirection
        {
            get { return _controllerDirection; }
            set
            {
                if (value == _controllerDirection)
                    return;

                _controllerDirection = value;

                switch (_controllerDirection)
                {
                    case ControllerDirection.Direct:
                        _kp = Math.Abs(_kp);
                        _ki = Math.Abs(_ki);
                        _kd = Math.Abs(_kd);
                        break;
                    case ControllerDirection.Reverse:
                        _kp = Math.Abs(_kp)*-1;
                        _ki = Math.Abs(_ki)*-1;
                        _kd = Math.Abs(_kd)*-1;
                        break;
                }
            }
        }

        /// <summary>
        ///     Gets the current controller mode. Sets whether the PID controller will be controlling
        ///     the process value (AUTOMATIC) or whether it will be manually controlled by an external process (MANUAL).
        /// </summary>
        public ControllerMode ControllerMode
        {
            get { return _controllerMode; }
            set
            {
                if (_controllerMode == ControllerMode.Manual && value == ControllerMode.Automatic)
                    Initialize();

                _controllerMode = value;
            }
        }

        /// <summary>
        ///     Gets or sets the proportional gain on the PID controller.
        /// </summary>
        public float ProportionalGain
        {
            get { return _proportionalGain; }
            set
            {
                if (value < 0)
                    _proportionalGain = 0;

                _kp = _proportionalGain;

                if (ControllerDirection == ControllerDirection.Reverse)
                    _kp = 0 - _kp;
            }
        }

        /// <summary>
        ///     Gets or sets the integral gain on the PID controller.
        /// </summary>
        public float IntegralGain
        {
            get { return _integralGain; }
            set
            {
                if (value < 0)
                    _integralGain = value;

                _ki = _integralGain*(float) SamplingRate.TotalSeconds;

                if (ControllerDirection == ControllerDirection.Reverse)
                    _ki = 0 - _integralGain;
            }
        }

        /// <summary>
        ///     Gets or sets the derivative gain on the PID controller.
        /// </summary>
        public float DerivativeGain
        {
            get { return _derivativeGain; }
            set
            {
                if (value < 0)
                    _derivativeGain = value;

                _kd = _derivativeGain/(float) SamplingRate.TotalSeconds;

                if (ControllerDirection == ControllerDirection.Reverse)
                    _kd = 0 - _derivativeGain;
            }
        }

        /// <summary>
        ///     Adjusts the minimum and maximum value that the PID controller can drive the output to.
        /// </summary>
        /// <param name="minimum">The minimum value the output can be driven to.</param>
        /// <param name="maximum">The maximum value the output can be driven to.</param>
        public void SetOutputLimits(float minimum, float maximum)
        {
            if (minimum > maximum)
                maximum = minimum;

            OutputMinimum = minimum;
            OutputMaximum = maximum;

            if (ControllerMode == ControllerMode.Manual)
                return;

            if (_readOutput() < OutputMinimum)
                _writeOutput(OutputMinimum);
            else if (_readOutput() > OutputMaximum)
                _writeOutput(OutputMaximum);

            if (_iTerm < OutputMinimum)
                _iTerm = OutputMinimum;
            else if (_iTerm > OutputMaximum)
                _iTerm = OutputMaximum;
        }

        /// <summary>
        ///     Starts a threaded timer to run the PID controller at the interval specified in <see cref="SamplingRate" />. The
        ///     output will be driven by the algorithm at the specified interval.
        /// </summary>
        public void Run()
        {
            if (IsRunning)
                return;

            IsRunning = true;
            _computeTimer = new Timer(callback => Compute(), null, TimeSpan.FromMilliseconds(0), SamplingRate);
        }

        /// <summary>
        ///     Computes a new output value to drive a process value to a given set-point.
        /// </summary>
        private void Compute()
        {
            if (ControllerMode == ControllerMode.Manual) return;

            var input = _readProcess();
            var error = _readSetPoint() - input;

            //Improvement suggested in issue 23
            //https://github.com/br3ttb/Arduino-PID-Library/issues/23
            _iTerm += _ki*(error + _lastError)/2.0f;

            var dInput = input - _lastInput;
            var output = _kp*error + _iTerm - _kd*dInput;

            //Improvement suggested in comment regarding anti windup.
            //http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/#comment-18721
            if (output > OutputMaximum)
            {
                _iTerm -= output - OutputMaximum;
                output = OutputMaximum;
            }
            else if (output < OutputMinimum)
            {
                _iTerm += OutputMinimum - output;
                output = OutputMinimum;
            }

            _writeOutput(output);
            _lastInput = input;
            _lastError = error;
        }

        /// <summary>
        ///     Performs functions to ensure a bumpless transfer from a manual control mode to automatic control mode.
        /// </summary>
        private void Initialize()
        {
            _iTerm = _readOutput();
            _lastInput = _readProcess();

            if (_iTerm > OutputMaximum)
                _iTerm = OutputMaximum;
            else if (_iTerm < OutputMinimum)
                _iTerm = OutputMinimum;
        }
    }
}