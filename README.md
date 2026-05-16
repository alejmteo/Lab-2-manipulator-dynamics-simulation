# Lab-2-manipulator-dynamics-simulation
## Introducción

El objetivo de esta práctica es implementar en una simulación la dinámica de un manipulador simple con ayuda de ROS2.

Además se van a alterar algunos parámetros del manipulador para observar cómo afectan en su dinámica con ayuda de algunas gráficas.

## Fundamentos teóricos

La ecuación con la cual se implementará la dinámica del manipulador viene dada por:

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q) = \tau + \tau_{ext}$$

Donde:

*   $q \in \mathbb{R}^{n \times 1}$ es el vector de posiciones articulares (`joint_positions_`).
*   $\dot{q} \in \mathbb{R}^{n \times 1}$ es el vector de velocidades articulares (`joint_velocities_`).
*   $\ddot{q} \in \mathbb{R}^{n \times 1}$ es el vector de aceleraciones articulares (`joint_accelerations_`).
*   $M(q) \in \mathbb{R}^{n \times n}$ es la matriz de inercia.
*   $C(q,\dot{q}) \in \mathbb{R}^{n \times n}$ es la matriz de fuerzas centrífugas y de Coriolis.
*   $F_b \in \mathbb{R}^{n \times n}$ es la matriz de fricción viscosa.
*   $g(q) \in \mathbb{R}^{n \times 1}$ es el vector de gravedad.
*   $\tau \in \mathbb{R}^{n \times 1}$ es el vector de pares articulares comandados (`joint_torques_`).
*   $\tau_{ext} \in \mathbb{R}^{n \times 1}$ es el vector de pares articulares debidos a fuerzas externas.

En este caso, al tener un manipulador de 2 grados de libertad (DoF)8($n=2$), la aceleración debida a los pares aplicados se define como:

$$\ddot{q} = M^{-1}(q)[\tau + \tau_{ext} - C(q,\dot{q})\dot{q} - F_b\dot{q} - g(q)]$$

Para calcular las aceleraciones articulares, primero se necesita calcular las matrices. Estas se pueden obtener aplicando las formulaciones de Lagrange o de Newton-Euler. En este caso, las matrices se definen de la siguiente manera:

**Matriz de Inercia, $M(q)$:**

$$M(q) = \begin{bmatrix} m_1 \cdot l_1^2 + m_2 \cdot (l_1^2 + 2 \cdot l_1 \cdot l_2 \cdot \cos(q_2) + l_2^2) & m_2 \cdot (l_1 \cdot l_2 \cdot \cos(q_2) + l_2^2) \\ m_2 \cdot (l_1 \cdot l_2 \cdot \cos(q_2) + l_2^2) & m_2 \cdot l_2^2 \end{bmatrix}$$

**Vector de Coriolis y fuerzas centrífugas, $C(q,\dot{q})\dot{q}$:**

$$C(q,\dot{q})\dot{q} = \begin{bmatrix} -m_2 \cdot l_1 \cdot l_2 \cdot \sin(q_2) \cdot (2 \cdot \dot{q}_1 \cdot \dot{q}_2 + \dot{q}_2^2) \\ m_2 \cdot l_1 \cdot l_2 \cdot \dot{q}_1^2 \cdot \sin(q_2) \end{bmatrix}$$

**Matriz de Fricción viscosa, $F_b$:**

$$F_b = \begin{bmatrix} b_1 & 0 \\ 0 & b_2 \end{bmatrix}$$

**Vector de Gravedad, $g(q)$:**

$$g(q) = \begin{bmatrix} (m_1 + m_2) \cdot l_1 \cdot g \cdot \cos(q_1) + m_2 \cdot g \cdot l_2 \cdot \cos(q_1 + q_2) \\ m_2 \cdot g \cdot l_2 \cdot \cos(q_1 + q_2) \end{bmatrix}$$

También se necesita calcular el Jacobiano para incluir en nuestro modelo las fuerzas/pares (wrenches) externos aplicados en el efector final (EE):

**Matriz Jacobiana, $J(q)$:**

$$J(q) = \begin{bmatrix} -l_1 \cdot \sin(q_1) - l_2 \cdot \sin(q_1 + q_2) & -l_2 \cdot \sin(q_1 + q_2) \\ l_1 \cdot \cos(q_1) + l_2 \cdot \cos(q_1 + q_2) & l_2 \cdot \cos(q_1 + q_2) \end{bmatrix}$$

Finalmente, se calcula calcular el par externo $\tau_{ext}$ como:

$$\tau_{ext} = J(q)^T \cdot F_{ext}$$

Además al estar implementando un sistema discreto, se debe discretizar la fórmula, quedando en consecuencia: 

$$\ddot{q}_{k+1} = M^{-1}(q_k)[\tau_k + \tau_{ext,k} - C(q_k,\dot{q}_k)\dot{q}_k - F_b\dot{q}_k - g(q_k)]$$

De la misma forma, se debe discreticar la velocidad y la posición:

$$\dot{q} = \int \ddot{q} dt \implies \dot{q}_{k+1} = \dot{q}_k + \ddot{q}_{k+1} \cdot \Delta t$$

$$q = \int \dot{q} dt \implies q_{k+1} = q_k + \dot{q}_{k+1} \cdot \Delta t$$



La aceleración se implementan de la siguiente forma en el código:

```cpp
// Initialize M, C, Fb, g_vec, J, and tau_ext
Eigen::MatrixXd M(2, 2);
Eigen::VectorXd C(2);
Eigen::MatrixXd Fb(2, 2);
Eigen::VectorXd g_vec(2);
Eigen::MatrixXd J(2, 2);
Eigen::VectorXd tau_ext(2);

// Initialize q1, q2, q_dot1, and q_dot2
double q1 = joint_positions_(0);
double q2 = joint_positions_(1);
double q_dot1 = joint_velocities_(0);
double q_dot2 = joint_velocities_(1);

// Placeholder calculations for M, C, Fb, g, and tau_ext
// Calculate matrix M
M(0, 0) = m1_ * pow(l1_, 2) + m2_ * (pow(l1_, 2) + 2 * l1_ * l2_ * cos(q2) + pow(l2_, 2));
M(0, 1) = m2_ * (l1_ * l2_ * cos(q2) + pow(l2_, 2));
M(1, 0) = M(0, 1);
M(1, 1) = m2_ * pow(l2_, 2);

// Calculate vector C (C is 2x1 because it already includes q_dot)
C << -m2_ * l1_ * l2_ * sin(q2) * (2 * q_dot1 * q_dot2 + pow(q_dot2, 2)),
m2_ * l1_ * l2_ * pow(q_dot1, 2) * sin(q2);

// Calculate Fb matrix
Fb << b1_, 0.0,
0.0, b2_;

// Calculate g_vect
g_vec << (m1_ + m2_) * l1_ * g_ * cos(q1) + m2_ * g_ * l2_ * cos(q1 + q2),
    m2_ * g_ * l2_ * cos(q1 + q2);

// Calculate J
J << -l1_ * sin(q1) - l2_ * sin(q1 + q2), -l2_ * sin(q1 + q2),
    l1_ * cos(q1) + l2_ * cos(q1 + q2), l2_ * cos(q1 + q2);

// Calculate tau_ext
tau_ext << J.transpose() * external_wrenches_;

// Calculate joint acceleration using the dynamic model: M * q_ddot = torque - C * q_dot - Fb * joint_velocities_ - g + tau_ext
Eigen::VectorXd q_ddot(2);
q_ddot << M.inverse() * (joint_torques_ - C - Fb * joint_velocities_ - g_vec + tau_ext);

return q_ddot;
```
Para obtener la velocidad y la posición discretizados, se implementa de la siguiente forma:

```cpp
// Method to calculate joint velocity
Eigen::VectorXd calculate_velocity()
{
    // Placeholder for velocity calculation
    // Integrate velocity over the time step (elapsed_time_)
    Eigen::VectorXd q_dot = joint_velocities_ + joint_accelerations_ * elapsed_time_;

    return q_dot;
}

// Method to calculate joint position
Eigen::VectorXd calculate_position()
{
    // Placeholder for position calculation
    // Integrate position over the time step (elapsed_time_)
    Eigen::VectorXd q = joint_positions_ + joint_velocities_ * elapsed_time_;

    return q;
}


```


## Resultados

Se han realizado varias pruebas con diferentes parámetros para ver como se comporta el manipulador en cada caso.



### Prueba con parámetros por defecto
Para esta prueba se han usado los siguientes parámetros
*   `frequency`: 1000.0
*   `m1`: 3.0
*   `m2`: 2.0
*   `l1`: 1.0
*   `l2`: 0.6
*   `b1`: 5.0
*   `b2`: 5.0
*   `g`: 9.81
*   `q0`: [0.785398, -0.785398]


![Posición angular de las articulaciones](posicion.png)




### Prueba con masa del eslabón 1 = 50
Para esta prueba se han usado los siguientes parámetros
*   `frequency`: 1000.0
*   `m1`: 50.0
*   `m2`: 2.0
*   `l1`: 1.0
*   `l2`: 0.6
*   `b1`: 5.0
*   `b2`: 5.0
*   `g`: 9.81
*   `q0`: [0.785398, -0.785398]





### Prueba con masa del eslabón 2 = 10
Para esta prueba se han usado los siguientes parámetros
*   `frequency`: 1000.0
*   `m1`: 3.0
*   `m2`: 10.0
*   `l1`: 1.0
*   `l2`: 0.6
*   `b1`: 5.0
*   `b2`: 5.0
*   `g`: 9.81
*   `q0`: [0.785398, -0.785398]





### Prueba con fricción articulación 1 = 50
Para esta prueba se han usado los siguientes parámetros
*   `frequency`: 1000.0
*   `m1`: 3.0
*   `m2`: 2.0
*   `l1`: 1.0
*   `l2`: 0.6
*   `b1`: 50.0
*   `b2`: 5.0
*   `g`: 9.81
*   `q0`: [0.785398, -0.785398]





### Prueba con fricción articulación 2 = 50
Para esta prueba se han usado los siguientes parámetros
*   `frequency`: 1000.0
*   `m1`: 3.0
*   `m2`: 2.0
*   `l1`: 1.0
*   `l2`: 0.6
*   `b1`: 5.0
*   `b2`: 50.0
*   `g`: 9.81
*   `q0`: [0.785398, -0.785398]





### Prueba con gravedad = 19.81
Para esta prueba se han usado los siguientes parámetros
*   `frequency`: 1000.0
*   `m1`: 3.0
*   `m2`: 2.0
*   `l1`: 1.0
*   `l2`: 0.6
*   `b1`: 5.0
*   `b2`: 5.0
*   `g`: 19.81
*   `q0`: [0.785398, -0.785398]



