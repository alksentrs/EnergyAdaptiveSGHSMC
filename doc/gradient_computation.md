# Gradient Computation in Energy-Adaptive SGHSMC

This document explains the computation of the negative gradient of the log posterior probability density function (−∇log(π(xi))) in the Energy-Adaptive SGHSMC particle filter implementation.

## Mathematical Background

The posterior probability density function (PDF) π(xi) can be written as:

```
π(xi) ∝ p(y|xi) * p(xi|x_prev)
```

where:
- p(y|xi) is the measurement likelihood
- p(xi|x_prev) is the state transition probability

Taking the negative log:

```
-log(π(xi)) = -log(p(y|xi)) - log(p(xi|x_prev))
```

## Implementation Details

### 1. Measurement Likelihood Term

For a Gaussian measurement model:

```
p(y|xi) = N(y; h(xi), R)
```

where:
- N(·) is the normal distribution
- h(xi) is the measurement function
- R is the measurement noise covariance

The negative gradient of the log-likelihood term is:

```
-∇log(p(y|xi)) = -∇[-(y-h(xi))'R^(-1)(y-h(xi))/2]
               = R^(-1)(y-h(xi))
```

In the code, this is implemented as:

```matlab
h_x = h(x);  % Get predicted measurements
measurement_error = y - h_x;  % Compute measurement innovation
grad_measurement = -params.R \ measurement_error;  % Weighted by inverse measurement noise
```

The `\` operator in MATLAB is the matrix left division, equivalent to multiplying by the inverse of the matrix. So `params.R \ measurement_error` is equivalent to `inv(params.R) * measurement_error`.

### 2. State Transition Term

The state transition term follows a similar logic:

```matlab
state_error = x - x_prev;
grad_state = 2 * params.alpha * state_error;
```

This comes from the negative gradient of the log of the state transition probability, which is typically modeled as a Gaussian centered at the previous state with covariance related to the adaptive parameter α.

### Complete Gradient

The complete gradient is the sum of these terms:

```matlab
grad_U = grad_measurement + grad_state;
```

## Usage in SGHSMC Update

The gradient computation is used in the SGHSMC update step:

```matlab
for i = 1:params.m
    xi = xi + params.epsilon * (Mmat \ ri);
    grad_U = compute_gradient(xi, y, x(:,m), params);
    ri = ri - params.epsilon * grad_U - params.epsilon * params.C * (Mmat \ ri) + ...
         sqrt(2 * (params.C - params.B) * params.epsilon) * randn(6,1);
end
```

## Key Parameters

- `params.R`: Measurement noise covariance matrix
- `params.alpha0`: Base adaptation rate
- `params.gamma1`: Adaptation sensitivity parameter
- `params.alpha`: Adaptive parameter computed as `params.alpha0 * exp(-params.gamma1 * norm(y - h(x_pred)))`

## References

This implementation is based on the paper:
> Kang, C. H., & Kim, S. Y. (2023). Energy-Adaptive SGHSMC: A Particle-Efficient Nonlinear Filter for High-Maneuver Target Tracking. Sejong University & Kunsan National University. 