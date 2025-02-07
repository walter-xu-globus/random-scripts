#pragma once
#include <algorithm>

class IncrementalPID
{
public:
    /**
     * @param kp       Proportional gain, K_p
     * @param ki       Integral gain, K_i
     * @param kd       "Derivative" gain for the incremental form
     *                 (Note: applied as (kd / kp) in the derivative term)
     * @param a        Filter constant for the derivative,
     *                 used in alpha = dt/(a * kd/kp + dt).
     */
    IncrementalPID(double kp, double ki, double kd, double a = 0.1)
      : m_kp(kp)
      , m_ki(ki)
      , m_kd(kd)
      , m_tau(a * kd / kp)
      , m_prevOutput(0.0)
      , m_prevError(0.0)
      , m_eFilteredPrev(0.0)
      , m_eFilteredPrev2(0.0)
    {
    }

    /**
     * Perform one PID update step (incremental form).
     *
     * @param error  The current error, e_k
     * @param period The timestep since the last update
     * @return       The new control output, u(t_k)
     */
    double calculate(double error, TimeSpan period)
    {
        const double dt = period.seconds<double>();

        if (dt <= 0.0) {
            // If dt is invalid, just return the previous output unchanged.
            return m_prevOutput;
        }

        //=== 1) Low-pass filter the error for the derivative term ===
        //     e'_k = alpha * e_k + (1 - alpha)* e'_{k-1}
        //     alpha = dt / (tau + dt)
        double alpha = dt / (m_tau + dt);
        double eFiltered = alpha * error + (1.0 - alpha) * m_eFilteredPrev;

        //=== 2) Compute each incremental PID component ===

        //--- (a) Incremental P-term: Kp * (e_k - e_{k-1})
        double pTerm = m_kp * (error - m_prevError);

        //--- (b) Incremental I-term: Ki * (dt/2) * (e_k + e_{k-1})
        double iTerm = m_ki * 0.5 * (error + m_prevError) * dt;

        //--- (c) Filtered D-term (increment):
        //      (Kd / Kp) * [ (e'_k - 2 e'_{k-1} + e'_{k-2}) / dt ]
        double dTerm = 0.0;
        if (m_kd != 0.0) {
            dTerm = (m_kd / m_kp) *
                    ((eFiltered - 2.0*m_eFilteredPrev + m_eFilteredPrev2) / dt);
        }

        //=== 3) New output = old output + (P-increment + I-increment + D-increment) ===
        double output = m_prevOutput + pTerm + iTerm + dTerm;

        //=== 4) Update states for next iteration ===
        m_prevOutput      = output;
        m_prevError       = error;
        m_eFilteredPrev2  = m_eFilteredPrev;  // e'_{k-2} <- e'_{k-1}
        m_eFilteredPrev   = eFiltered;        // e'_{k-1} <- e'_{k}

        //=== 5) Return the new controller output ===
        return output;
    }

private:
    double m_kp;         // K_p
    double m_ki;         // K_i
    double m_kd;         // K_d
    double m_tau;        // Filter time constant, for derivative filtering

    double m_prevOutput;       // u(t_{k-1})
    double m_prevError;        // e_{k-1}
    double m_eFilteredPrev;    // e'_{k-1}
    double m_eFilteredPrev2;   // e'_{k-2}
};
