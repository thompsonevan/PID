import java.lang.Math;

public class PID {
    private static int instances;

    // Factor for "proportional" control
    private double m_Kp;
  
    // Factor for "integral" control
    private double m_Ki;
  
    // Factor for "derivative" control
    private double m_Kd;
  
    // The period (in seconds) of the loop that calls the controller
    private final double m_period;
  
    private double m_maximumIntegral = 1.0;
  
    private double m_minimumIntegral = -1.0;
  
    // Maximum input - limit setpoint to this
    private double m_maximumInput;
  
    // Minimum input - limit setpoint to this
    private double m_minimumInput;
  
    // Input range - difference between maximum and minimum
    private double m_inputRange;
  
    // Do the endpoints wrap around? eg. Absolute encoder
    private boolean m_continuous;
  
    // The error at the time of the most recent call to calculate()
    private double m_positionError;
    private double m_velocityError;
  
    // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
    private double m_prevError;
  
    // The sum of the errors for use in the integral calc
    private double m_totalError;
  
    // The percentage or absolute error that is considered at setpoint.
    private double m_positionTolerance = 0.05;
    private double m_velocityTolerance = Double.POSITIVE_INFINITY;
  
    private double m_setpoint;
  

    public PID(double Kp, double Ki, double Kd) {
      this(Kp, Ki, Kd, 0.02);
    }
  

    public PID(double Kp, double Ki, double Kd, double period) {
      m_Kp = Kp;
      m_Ki = Ki;
      m_Kd = Kd;
  
      m_period = period;
  
      instances++;
    }

    public double clamp(double value, double low, double high){
        return Math.max(low, Math.min(value, high));
    }

    public void setPID(double Kp, double Ki, double Kd) {
      m_Kp = Kp;
      m_Ki = Ki;
      m_Kd = Kd;
    }
  


    public void setP(double Kp) {
      m_Kp = Kp;
    }
  

    public void setI(double Ki) {
      m_Ki = Ki;
    }
  

    public void setD(double Kd) {
      m_Kd = Kd;
    }
  

    public double getP() {
      return m_Kp;
    }
  

    public double getI() {
      return m_Ki;
    }

    public double getD() {
      return m_Kd;
    }
  

    public double getPeriod() {
      return m_period;
    }
  

    public void setSetpoint(double setpoint) {
      if (m_maximumInput > m_minimumInput) {
        m_setpoint = clamp(setpoint, m_minimumInput, m_maximumInput);
      } else {
        m_setpoint = setpoint;
      }
    }
  
    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
      return m_setpoint;
    }
  
    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * SetTolerance. This asssumes that the maximum and minimum input were set using SetInput.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
      return Math.abs(m_positionError) < m_positionTolerance
          && Math.abs(m_velocityError) < m_velocityTolerance;
    }
  
    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers
     * them to be the same point and automatically calculates the shortest route
     * to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
      m_continuous = true;
      setInputRange(minimumInput, maximumInput);
    }
  
    /**
     * Disables continuous input.
     */
    public void disableContinuousInput() {
      m_continuous = false;
    }
  
    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>When the cap is reached, the integrator value is added to the controller
     * output rather than the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
      m_minimumIntegral = minimumIntegral;
      m_maximumIntegral = maximumIntegral;
    }
  
    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
      setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }
  
    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
      m_positionTolerance = positionTolerance;
      m_velocityTolerance = velocityTolerance;
    }
  
    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
      return getContinuousError(m_positionError);
    }
  
    /**
     * Returns the velocity error.
     */
    public double getVelocityError() {
      return m_velocityError;
    }
  
    public double calculate(double measurement, double setpoint) {
      // Set setpoint to provided value
      setSetpoint(setpoint);
      return calculate(measurement);
    }
  
    public double calculate(double measurement) {
      m_prevError = m_positionError;
      m_positionError = getContinuousError(m_setpoint - measurement);
      m_velocityError = (m_positionError - m_prevError) / m_period;
  
      if (m_Ki != 0) {
        m_totalError = clamp(m_totalError + m_positionError * m_period,
            m_minimumIntegral / m_Ki, m_maximumIntegral / m_Ki);
      }
  
      return m_Kp * m_positionError + m_Ki * m_totalError + m_Kd * m_velocityError;
    }
  
    public void reset() {
      m_prevError = 0;
      m_totalError = 0;
    }
  
    protected double getContinuousError(double error) {
      if (m_continuous && m_inputRange > 0) {
        error %= m_inputRange;
        if (Math.abs(error) > m_inputRange / 2) {
          if (error > 0) {
            return error - m_inputRange;
          } else {
            return error + m_inputRange;
          }
        }
      }
      return error;
    }
  
    private void setInputRange(double minimumInput, double maximumInput) {
      m_minimumInput = minimumInput;
      m_maximumInput = maximumInput;
      m_inputRange = maximumInput - minimumInput;
  

      if (m_maximumInput > m_minimumInput) {
        m_setpoint = clamp(m_setpoint, m_minimumInput, m_maximumInput);
      }
    }
}