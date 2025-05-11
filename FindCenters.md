# Geometric Median using Weiszfeld’s Algorithm

## Definition

The **geometric median** of a set of points \( \{x_1, x_2, \dots, x_n\} \) in \( \mathbb{R}^d \) is the point \( y^* \) that minimizes the sum of Euclidean distances to all the points:

\[
y^* = \arg\min_y \sum_{i=1}^{n} \|x_i - y\|
\]

Unlike the centroid (arithmetic mean), it is **robust to outliers**.

---

##  Algorithm Used: Weiszfeld’s Algorithm

An iterative method for computing the geometric median.

### Step-by-step Explanation:

1. **Initialize** the estimate \( y \) as the **mean** of the input points \( X \):
   ```python
   y = np.mean(X, axis=0)
   ```

2. **Iterate**:
   - Compute distances \( D \) from the current estimate \( y \) to all points in \( X \):
     ```python
     D = np.linalg.norm(X - y, axis=1)
     ```
   - Exclude points that are too close (to avoid division by zero):
     ```python
     nonzero = D > eps
     ```
   - Compute **weights** \( W \) inversely proportional to distance:
     ```python
     W = 1 / D[nonzero]
     T = X[nonzero]
     ```
   - Update \( y \) using a **weighted average**:
     \[
     y_{\text{new}} = \frac{\sum_i w_i x_i}{\sum_i w_i}
     \]
     ```python
     y_new = (W[:, None] * T).sum(axis=0) / W.sum()
     ```

3. **Check for convergence**: Stop if the change in \( y \) is smaller than a threshold:
   ```python
   if np.linalg.norm(y - y_new) < eps:
       return y_new
   ```

4. Repeat for a maximum number of iterations or until convergence.

---

## Pseudocode Summary

```text
1. y = initial guess (mean)
2. repeat until convergence or max_iter:
   a. compute distance from y to all points
   b. compute inverse distance weights
   c. update y using weighted average
   d. stop if change < eps
```
