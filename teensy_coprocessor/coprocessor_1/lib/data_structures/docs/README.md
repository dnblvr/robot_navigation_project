

## Explanation of Madgwick Filter Implementation

it uses the reference gravitational vector and magnetic north vector to compute orientation.

### Gravity Gradient Calculation

given a reference gravitational vector in the earth-frame $\hat{g} = \begin{pmatrix}0 & 0 & 0& 1\end{pmatrix}$, the current estimated orientation quaternion $q$ is used to rotate this vector into the body-frame:
$$\hat{g}_b = q^* \hat{g} q \nabla $$
