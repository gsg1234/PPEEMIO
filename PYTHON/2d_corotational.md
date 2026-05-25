# 2D Corotational Beam Formulation

**Original author:** Louie L. Yaw, Walla Walla University (2009)  
**Primary reference:** Crisfield [3]

---

## 1. Introduction and Corotational Concept

The corotational method is an approach for **geometrically nonlinear analysis** that allows arbitrarily large global displacements and rotations while retaining a linear elastic behavior at the local element level. The core idea is to separate **rigid body motion** (global translation and rotation) from the **strain-producing deformations** of each element.

This is achieved by attaching a **local co-rotating reference frame** to each beam element. This frame translates and rotates with the element, so that in local coordinates only pure deformations remain — with no rigid body contribution.

The three essential ingredients of any corotational formulation are:

1. The **angle of rotation** of the co-rotating frame.
2. The **relations between global and local variables**.
3. The **variationally consistent tangent stiffness matrix**.

---

## 2. Element Kinematics

### 2.1 Initial and Current Length

For a beam element with nodes 1 and 2, the initial (undeformed) length is:

$$L_o = \sqrt{(X_2 - X_1)^2 + (Y_2 - Y_1)^2}$$

After applying nodal displacements $(u_i, w_i)$, the current (deformed) length becomes:

$$L = \sqrt{((X_2+u_2)-(X_1+u_1))^2 + ((Y_2+w_2)-(Y_1+w_1))^2}$$

### 2.2 Local Axial Deformation

The local axial displacement is computed as:

$$u_\ell = \frac{L^2 - L_o^2}{L + L_o}$$

This form is numerically better conditioned than the naive difference $L - L_o$ when length changes are small (as advocated by Crisfield). The axial strain and axial force follow directly:

$$\varepsilon_{x\ell} = \frac{u_\ell}{L_o}, \qquad N = \frac{EA\, u_\ell}{L_o}$$

### 2.3 Angle of the Co-rotating Frame

The initial angle of the element in the undeformed configuration:

$$\beta_o = \text{atan2}(Y_2 - Y_1,\ X_2 - X_1)$$

The current angle in the deformed configuration:

$$\beta = \text{atan2}\!\left(Y_2+w_2-(Y_1+w_1),\ X_2+u_2-(X_1+u_1)\right)$$

For efficiency, the sine and cosine of $\beta$ are computed directly from global variables without evaluating $\beta$ explicitly in most operations:

$$\cos\beta = \frac{(X_2+u_2)-(X_1+u_1)}{L}, \qquad \sin\beta = \frac{(Y_2+w_2)-(Y_1+w_1)}{L}$$

---

## 3. Flexural Deformations

The **local nodal rotations** $\theta_{1\ell}$ and $\theta_{2\ell}$ are obtained by subtracting the rigid body rotation of the element from the global nodal rotations:

$$\theta_{1\ell} = \theta_1 + \beta_o - \beta, \qquad \theta_{2\ell} = \theta_2 + \beta_o - \beta$$

To allow arbitrarily large rotations (overcoming the $|\beta| < \pi/2$ limitation of the standard arctangent), these are reformulated using sines and cosines. Defining $\beta_1 = \theta_1 + \beta_o$ and $\beta_2 = \theta_2 + \beta_o$:

$$\sin\theta_{1\ell} = \cos\beta\sin\beta_1 - \sin\beta\cos\beta_1$$
$$\cos\theta_{1\ell} = \cos\beta\cos\beta_1 + \sin\beta\sin\beta_1$$

so that:

$$\theta_{1\ell} = \text{atan2}(\cos\beta\sin\beta_1 - \sin\beta\cos\beta_1,\ \cos\beta\cos\beta_1 + \sin\beta\sin\beta_1)$$

and analogously for $\theta_{2\ell}$. This is the key step that allows the 2D corotational element to handle arbitrarily large rotations.

### Local End Moments and Shear Force

The local end moments are obtained from standard Euler-Bernoulli beam stiffness:

$$\begin{Bmatrix} \bar{M}_1 \\ \bar{M}_2 \end{Bmatrix} = \frac{2EI}{L_o} \begin{bmatrix} 2 & 1 \\ 1 & 2 \end{bmatrix} \begin{Bmatrix} \theta_{1\ell} \\ \theta_{2\ell} \end{Bmatrix}$$

The transverse shear force is recovered by static equilibrium (assuming no distributed loads between nodes):

$$V_1 = \frac{\bar{M}_1 + \bar{M}_2}{L}, \qquad V_2 = -V_1$$

---

## 4. Relation Between Global and Local Variables

For a virtual displacement $\delta\mathbf{p}$ in global coordinates (a 6-component vector: $u_1, w_1, \theta_1, u_2, w_2, \theta_2$), the local virtual deformations are related by:

$$\delta\mathbf{p}_\ell = \mathbf{B}\,\delta\mathbf{p}$$

where the transformation matrix $\mathbf{B}$ (3×6) is:

$$\mathbf{B} = \begin{bmatrix} -c & -s & 0 & c & s & 0 \\ -s/L & c/L & 1 & s/L & -c/L & 0 \\ -s/L & c/L & 0 & s/L & -c/L & 1 \end{bmatrix}$$

with $c = \cos\beta$ and $s = \sin\beta$. This matrix maps infinitesimal local deformations to global nodal displacements and is derived from geometric arguments using the dot products of unit vectors along and perpendicular to the current element axis.

### Global Internal Forces

By equivalence of virtual work in the local and global systems, the global nodal internal forces for element $i$ are:

$$\mathbf{q}_i = \mathbf{B}^T \mathbf{q}_{\ell i}$$

where $\mathbf{q}_{\ell i} = [N,\ \bar{M}_1,\ \bar{M}_2]^T$ are the local internal forces. This is straightforward to compute once the local forces are known from the updated member data.

---

## 5. Variationally Consistent Tangent Stiffness Matrix

The total tangent stiffness matrix is the sum of a **material** (or transformed) part and a **geometric** part:

$$\mathbf{k}_t = \mathbf{k}_{t1} + \mathbf{k}_{t\sigma}$$

### 5.1 Transformed Material Stiffness

$$\mathbf{k}_{t1} = \mathbf{B}^T \mathbf{C}_\ell \mathbf{B}$$

where $\mathbf{C}_\ell$ is the local element stiffness matrix, expressed in terms of $EA/L_o$ and the radius of gyration $r = \sqrt{I/A}$:

$$\mathbf{C}_\ell = \frac{EA}{L_o} \begin{bmatrix} 1 & 0 & 0 \\ 0 & 4r^2 & 2r^2 \\ 0 & 2r^2 & 4r^2 \end{bmatrix}$$

### 5.2 Geometric Stiffness

The geometric stiffness matrix accounts for second-order effects (tension stiffening and compression softening):

$$\mathbf{k}_{t\sigma} = \frac{N}{L}\mathbf{z}\mathbf{z}^T + \frac{\bar{M}_1+\bar{M}_2}{L^2}(\mathbf{r}\mathbf{z}^T + \mathbf{z}\mathbf{r}^T)$$

where the auxiliary vectors are:

$$\mathbf{r}^T = \begin{bmatrix} -c & -s & 0 & c & s & 0 \end{bmatrix}$$
$$\mathbf{z}^T = \begin{bmatrix} s & -c & 0 & -s & c & 0 \end{bmatrix}$$

Note that $\mathbf{r}\mathbf{z}^T$ and $\mathbf{z}\mathbf{r}^T$ are outer (tensor) products. The geometric stiffness is derived by taking the variation of $\mathbf{B}^T$ and is essential for capturing buckling and large-deflection behavior.

---

## 6. Load Control Algorithm with Newton-Raphson Iterations

The following algorithm solves the nonlinear equilibrium problem using an **incremental-iterative load control** strategy. At each load increment, Newton-Raphson iterations drive the residual (force imbalance) to zero.

### 6.1 Initialization

```
Define and initialize:
  β_o  ← initial angles of each beam member, eq. (8)
  A, E, I ← cross-sectional properties for each member
  F    ← total vector of externally applied global nodal forces
  u    ← global nodal displacement vector = 0
  q_ℓ  ← local force storage vector (N, M̄₁, M̄₂ per member) = 0
  x, y ← nodal coordinates in the undeformed configuration
  L_o  ← initial element lengths, eq. (1)
```

### 6.2 Loop Over Load Increments

**For** $n = 0, 1, \ldots, n_{inc}-1$:

**(a) Load factor and incremental force vector**
```
λ  = 1 / n_inc
dF = λ · F
```

**(b) Assemble global tangent stiffness matrix**
```
For each element i:
    Compute c_i, s_i, L_i  using current u
    Compute k_t1_i = Bᵢᵀ Cℓ_i Bᵢ
    Compute k_tσ_i  using current N_i, M̄₁_i, M̄₂_i
    k_i = k_t1_i + k_tσ_i

K = assemble(k_i)    ← global tangent stiffness
```

**(c) Apply boundary conditions**
```
K_s = apply_supports(K)
    ← zero rows/cols for constrained DOFs; set diagonal to 1
```

**(d) Solve for incremental displacements**
```
du = K_s⁻¹ · dF
```

**(e–f) Update displacements and applied forces**
```
u^{n+1} = u^n + du
F^{n+1} = F^n + dF
```

**(g–h) Update member data and internal force vector**
```
Call member update routine with u_current = u^{n+1}  (see §6.4)
Build F_int^{n+1} from updated q_ℓ                   (see §6.5)
```

**(i–j) Compute residual**
```
R      = F_int^{n+1} - F^{n+1}
Modify R to account for supported DOFs
R_norm = sqrt(R · R)
```

### 6.3 Newton-Raphson Equilibrium Iterations

Set iteration variables:
```
k         = 0
tolerance = 1e-3
maxiter   = 100
δu        = 0
q_ℓ_temp  = q_ℓ^{n+1}
```

**While** $R_{norm} > tolerance$ **and** $k < maxiter$:

```
i.    Recompute K using current c, s, L, L_o, A, E, I, q_ℓ_temp → K_s
ii.   Apply boundary conditions → K_s
iii.  Compute correction:  δu^{k+1} = δu^k - K_s⁻¹ · R
      (note: u^{n+1} is NOT updated until all iterations are complete)
iv.   u_current = u^{n+1} + δu^{k+1}
v.    Call member update routine with u_current      (see §6.4)
vi.   Rebuild F_int^{n+1} from q_ℓ_temp             (see §6.5)
vii.  R      = F_int^{n+1} - F^{n+1}
      Modify R for supported DOFs
viii. R_norm = sqrt(R · R)
ix.   k = k + 1
```

**End While**

**Finalize increment:**
```
q_ℓ^{n+1}      = q_ℓ^k_temp
u_final^{n+1}  = u^{n+1} + δu_{(k)}
```

### 6.4 Member Data Update Routine

Called with the current displacement vector $\mathbf{u}_c$. Loops over each element $i$:

```
Step 1 — Compute nodal distance components:
    dX_i = (X₂ + u₂) - (X₁ + u₁)
    dY_i = (Y₂ + w₂) - (Y₁ + w₁)

Step 2 — Update current length:
    L_i = sqrt(dX_i² + dY_i²)

Step 3 — Update cosine and sine of current angle:
    cos β_i = dX_i / L_i
    sin β_i = dY_i / L_i

Step 4 — Compute local axial displacement (well-conditioned form):
    u_ℓ_i = (L_i² - L_o_i²) / (L_i + L_o_i)

Step 5 — Compute axial force:
    N_i = E_i · A_i · u_ℓ_i / L_o_i

Step 6 — Compute current angle (for local rotations):
    β_i = atan2(dY_i, dX_i)

Step 7 — Compute intermediate angles:
    β₁ = θ₁ + β_o_i
    β₂ = θ₂ + β_o_i

Step 8 — Compute local nodal rotations (large-rotation safe):
    θ_1ℓ = atan2( cosβ·sinβ₁ - sinβ·cosβ₁,
                  cosβ·cosβ₁ + sinβ·sinβ₁ )
    θ_2ℓ = atan2( cosβ·sinβ₂ - sinβ·cosβ₂,
                  cosβ·cosβ₂ + sinβ·sinβ₂ )

Step 9 — Compute local end moments:
    {M̄₁, M̄₂} = (2EI/L_o) · [2 1; 1 2] · {θ_1ℓ, θ_2ℓ}

Step 10 — Store updated local forces:
    q_ℓ_i = [N_i, M̄₁_i, M̄₂_i]ᵀ
    Insert into global storage vector q_ℓ
```

### 6.5 Internal Force Vector Assembly

For each element $i$, transform local forces to global coordinates and assemble:

```
q_i    = Bᵢᵀ · q_ℓi          ← global internal forces for element i

F_int  = Assemble(q_i)        ← accumulate contributions at shared DOFs
```

The assembly follows the standard finite element procedure: each component of $\mathbf{q}_i$ is added to the global position corresponding to the element's degree of freedom mapping.

---

## 7. Implementation Notes

- Always use `atan2` instead of `atan` for angle calculations — it correctly handles the full range $(-\pi, \pi]$ and is essential for large-rotation problems.
- The geometric stiffness $\mathbf{k}_{t\sigma}$ must be included to correctly capture **compression softening** (buckling) and **tension stiffening**.
- Standard load control fails for problems with **snap-through** or **snap-back** behavior (e.g., Lee's frame). These require arc-length control or generalized displacement control methods.
- The formulation assumes **small local strains** but places no restriction on the magnitude of global rotations or displacements.
- The convergence tolerance on the residual norm is typically $10^{-3}$, with a maximum of 100 Newton-Raphson iterations per increment.

---

## 8. Key Variable Reference

| Symbol | Description |
|---|---|
| $L_o$, $L$ | Initial and current element length |
| $\beta_o$, $\beta$ | Initial and current element axis angle |
| $u_\ell$ | Local axial displacement |
| $\theta_{1\ell}$, $\theta_{2\ell}$ | Local nodal rotations (rigid-body-free) |
| $N$ | Axial force |
| $\bar{M}_1$, $\bar{M}_2$ | Local end moments |
| $\mathbf{B}$ | Local-to-global transformation matrix (3×6) |
| $\mathbf{C}_\ell$ | Local element stiffness matrix (3×3) |
| $\mathbf{k}_{t1}$ | Transformed material stiffness (6×6) |
| $\mathbf{k}_{t\sigma}$ | Geometric stiffness matrix (6×6) |
| $\mathbf{K}$ | Assembled global tangent stiffness |
| $\mathbf{R}$ | Residual vector (force imbalance) |
| $\mathbf{r}$, $\mathbf{z}$ | Auxiliary vectors for geometric stiffness |
