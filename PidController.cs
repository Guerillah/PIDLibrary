//This library is a port of Brett Beauregard's Arduino PID library with some variations:
//http://www.arduino.cc/playground/Code/PIDLibrary

using System;

namespace PID
{
    public class PidController
    {
        private readonly Func<float> _readOutput;
        private readonly Func<float> _readProcess;
        private readonly Func<float> _readSetPoint;
        private readonly Action<float> _writeOutput;
        private TimeSpan _computeRate;
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
        private float _outputMaximum;
        private float _outputMinimum;
        private float _proportionalGain;

        /// <summary>
        ///     Instantiates a new PID controller class.
        /// </summary>
        /// <param name="computeRate">
        ///     The rate at which the <see cref="Compute" /> method is intended to be called via an external
        ///     timer/thread/etc... important for recalculating internal Kd,Ki,Kp variables when compute rate changes.
        /// </param>
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
        ///     Thrown when <see cref="readProcess" /> or <see cref="readOutput" /> or
        ///     <see cref="writeOutput" /> or <see cref="readSetPoint" /> are null.
        /// </exception>
        public PidController(TimeSpan computeRate, float outputMinimum, float outputMaximum, Func<float> readProcess,
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

            ComputeRate = computeRate;
            OutputMinimum = outputMinimum;
            OutputMaximum = outputMaximum;
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
        ///     Gets or sets the maximum value the output can be written to.
        /// </summary>
        public float OutputMaximum
        {
            get { return _outputMaximum; }
            set
            {
                _outputMaximum = value;

                if (ControllerMode == ControllerMode.Manual)
                    return;

                if (_readOutput() > _outputMaximum)
                    _writeOutput(_outputMaximum);

                if (_iTerm > _outputMaximum)
                    _iTerm = _outputMaximum;
            }
        }

        /// <summary>
        ///     Gets or sets the minimum value the output can be written to. It must be lower than <see cref="OutputMaximum" />
        /// </summary>
        public float OutputMinimum
        {
            get { return _outputMinimum; }
            set
            {
                if (value >= OutputMaximum)
                    return;

                _outputMinimum = value;

                if (ControllerMode == ControllerMode.Manual)
                    return;

                if (_readOutput() < _outputMinimum)
                    _writeOutput(_outputMinimum);

                if (_iTerm < _outputMinimum)
                    _iTerm = _outputMinimum;
            }
        }

        /// <summary>
        ///     Gets or sets the rate at which the <see cref="Compute" /> method should be called via an external
        ///     timer/thread/etc...
        /// </summary>
        public TimeSpan ComputeRate
        {
            get { return _computeRate; }
            set
            {
                if (value.TotalMilliseconds <= 0)
                    throw new ArgumentException("Sampling rate must be greater than 0 ms.");

                if (value.TotalMilliseconds.Equals(_computeRate.TotalMilliseconds))
                    return;

                var samplingRateRatio = value.TotalMilliseconds / _computeRate.TotalMilliseconds;

                _computeRate = value;

                if (double.IsInfinity(samplingRateRatio))
                    return;

                _ki *= (float)samplingRateRatio;
                _kd /= (float)samplingRateRatio;
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

                _kp = 0 - _kp;
                _ki = 0 - _ki;
                _kd = 0 - _kd;

                _controllerDirection = value;
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
                    throw new ArgumentException("Proportional gain must greater than or equal to 0.", nameof(value));

                _proportionalGain = value;
                _kp = _proportionalGain;
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
                    throw new ArgumentException("Integral gain must greater than or equal to 0.", nameof(value));

                _integralGain = value;
                _ki = _integralGain * (float)ComputeRate.TotalSeconds;

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
                    throw new ArgumentException("Derivative gain must greater than or equal to 0.", nameof(value));

                _derivativeGain = value;
                _kd = _derivativeGain / (float)ComputeRate.TotalSeconds;

                if (ControllerDirection == ControllerDirection.Reverse)
                    _kd = 0 - _derivativeGain;
            }
        }

        /// <summary>
        ///     Computes a new output value to drive a process value to a set-point. This method should be called at the rate
        ///     specified in <see cref="ComputeRate" />
        /// </summary>
        public void Compute()
        {
            if (ControllerMode == ControllerMode.Manual) return;

            var input = _readProcess();
            var error = _readSetPoint() - input;

            //Improvement suggested in issue 23
            //https://github.com/br3ttb/Arduino-PID-Library/issues/23
            _iTerm += _ki * (error + _lastError) / 2.0f;

            var dInput = input - _lastInput;
            var output = _kp * error + _iTerm - _kd * dInput;

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

            if (_iTerm > _outputMaximum)
                _iTerm = _outputMaximum;
            else if (_iTerm < _outputMinimum)
                _iTerm = _outputMinimum;
        }
    }
}