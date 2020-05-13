State Observers
===============

State observers combine information about a system's behavior and external measurements to estimate the true state of the system. A common observer used for linear systems is the Kalman Filter.

.. important:: It is important to develop an intuition for what a Kalman filter is actually doing. The book `Kalman and Bayesian Filters in Python by Roger Labbe <https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python>`__ provides a great visual and interactive introduction to Bayesian filters. The Kalman filters in WPILib use linear algebra to gentrify the math, but the ideas are similar to the single-dimensional case. We suggest reading through Chapter 4 to gain an intuition for what these filters are doing.

To summarize, Kalman filters (and all Bayesian filters) have two parts: prediction and correction. Prediction projects our state estimate forward in time according to our system's dynamics, and correct steers the estimated state towards the measured state. While filters often preform both in the same timestep, it's not strictly necessary -- For example, WPILib's pose estimators call predict frequently, and correct only when new measurement data is available (for example, from a low-framerate vision system).

Kalman filters utilize `Gaussian distributions <https://en.wikipedia.org/wiki/Gaussian_function>`__ (or bell curves) to model the noise in a process.

The following shows the equations of a discrete-time Kalman filter:

.. math::
    \text{Predict step} \nonumber \\
    \hat{\mathbf{x}}_{k+1}^- &= \mathbf{A}\hat{\mathbf{x}}_k + \mathbf{B} \mathbf{u}_k \\
    \mathbf{P}_{k+1}^- &= \mathbf{A} \mathbf{P}_k^- \mathbf{A}^T +
        \mathbf{\Gamma}\mathbf{Q}\mathbf{\Gamma}^T \\
    \text{Update step} \nonumber \\
    \mathbf{K}_{k+1} &=
        \mathbf{P}_{k+1}^- \mathbf{C}^T (\mathbf{C}\mathbf{P}_{k+1}^- \mathbf{C}^T +
        \mathbf{R})^{-1} \\
    \hat{\mathbf{x}}_{k+1}^+ &=
        \hat{\mathbf{x}}_{k+1}^- + \mathbf{K}_{k+1}(\mathbf{y}_{k+1} -
        \mathbf{C} \hat{\mathbf{x}}_{k+1}^- - \mathbf{D}\mathbf{u}_{k+1}) \\
    \mathbf{P}_{k+1}^+ &= (\mathbf{I} - \mathbf{K}_{k+1}\mathbf{C})\mathbf{P}_{k+1}^-

.. math::
    \begin{tabular}{llll}
      $\mathbf{A}$ & system matrix & $\hat{\mathbf{x}}$ & state estimate vector \\
      $\mathbf{B}$ & input matrix       & $\mathbf{u}$ & input vector \\
      $\mathbf{C}$ & output matrix      & $\mathbf{y}$ & output vector \\
      $\mathbf{D}$ & feedthrough matrix & $\mathbf{\Gamma}$ & process noise intensity
        vector \\
      $\mathbf{P}$ & error covariance matrix & $\mathbf{Q}$ & process noise covariance
        matrix \\
      $\mathbf{K}$ & Kalman gain matrix & $\mathbf{R}$ & measurement noise covariance
        matrix
    \end{tabular}

In a system, covariance (short for correlated variance) is a measurement of how two variables are correlated. In s system with a single state, the covariance matrix is simply :math:`\mathbf{\text{cov}(x_1, x_1)}`, or a matrix containing the variance :math:`\mathbf{\text{var} x_1} of the state :math:`x_1`. The magnitude of this variance is the square of the standard deviation of the Gaussian function describing the current state estimate. Relatively large values for covariance might indicate noisy data, while small covariances might indicate that the filter is more confident about it's estimate. Remember that "large" and "small" values for variance or covariance are relative to the base unit being used -- for example, if :math:`\mathbf{x_1}` was measured in meters, :math:`\mathbf{\text{cov}(x_1, x_1)}` would be in meters squared.

Covariance matrices are written in the following form:

.. math::
  \mathbf{\Sigma} &= \begin{bmatrix}
    \text{cov}(x_1, x_1) & \text{cov}(x_1, x_2) & \ldots & \text{cov}(x_1, x_n) \\
    \text{cov}(x_2, x_1) & \text{cov}(x_2, x_2) & \ldots & \text{cov}(x_1, x_n) \\
    \vdots         & \vdots         & \ddots & \vdots \\
    \text{cov}(x_n, x_1) & \text{cov}(x_n, x_2) & \ldots & \text{cov}(x_n, x_n) \\
  \end{bmatrix}

The state estimate :math:`\mathbf{x}`, together with :math:`\mathbf{P}`, describe the mean and covariance of the Gaussian function that describes our estimate of the system's true state.

The error covariance matrix :math:`\mathbf{P}` describes the covariance of the state estimate :math:`\mathbf{\hat{x}}`. If :math:`\mathbf{P}` is large our uncertainty about the true state is large. Conversely, a :math:`\mathbf{P}` with smaller elements would imply less uncertainty about our true state. In the prediction step, :math:`\mathbf{P}` grows at a rate proportional to the process noise covariance :math:`\mathbf{Q}` and process noise intensity vector :math:`\mathbf{\Gamma}` to show how our certainty about the system's state decreases as we project the model forward. 

Predict step
------------

In prediction, our state estimate is updated according to the linear system dynamics :math:`\mathbf{\dot{x} = Ax + Bu}`. Furthermore, our error covariance :math:`\mathbf{P}` increases by the process noise covariance matrix :math:`\mathbf{Q}`. Larger values of :math:`\mathbf{Q}` will make our error covariance :math:`\mathbf{P}` grow more quickly. This :math:`\mathbf{P}` is used in the correction step to weight the model and measurements.

Correct step
------------

In the correct step, our state estimate is updated to include new measurement information. This new information is weighted against the state estimate :math:`\mathbf{\hat{x}}` by the Kalman gain :math:`\mathbf{K}`. Large values of :math:`\mathbf{K}` more highly weight incoming measurements, while smaller values of :math:`\mathbf{K}` more highly weight our state prediction. Because :math:`\mathbf{K}` is related to :math:`\mathbf{P}`, larger values of :math:`\mathbf{P}` will increase :math:`\mathbf{K}` and more heavily weight measurements. If, for example, a filter is predicted for a long duration, the large :math:`\mathbf{P}` would heavily weight the new information.

Finally, the error covariance :math:`\mathbf{P}` decreases to increase our confidence in the state estimate. 

Tuning Kalman Filters
---------------------

WPILib's Kalman Filter classes' constructors take a linear system, a vector of process noise standard deviations and measurement noise standard deviations. These are converted to :math:`\mathbf{Q}` and :math:`\mathbf{R}` matrices by filling these diagonals with the square of the standard deviations,  or covariances, for each state. By decreasing a state's standard deviation (and therefore its corresponding entry in :math:`\mathbf{Q}`), the filter will distrust incoming measurements more. Similarly, increasing a state's standard deviation will trust incoming measurements more. The same holds for the measurement standard deviations -- decreasing an entry will make the filter more highly trust the incoming measurement for the corresponding state, while increasing it will decrease trust in the measurement.

Glossary
--------

.. glossary::

    Observer
        In control theory, a system that provides an estimate of the internal :term:`state` of a given real :term:`system` from measurements of the :term:`input` and :term:`output` of the real :term:`system`. WPILib includes a Kalman Filter class for observing linear systems, and ExtendedKalmanFilter and UnscentedKalmanFilter classes for nonlinear systems. 

