# GraphCut-PCD-Segmentation


The idea of the algorithm is as follows:

Given a point cloud, the algorithm constructs a graph where:
- Each point in the cloud is represented as a vertex.
- Two additional vertices are introduced: a **source** and a **sink**.

Every point (vertex) is connected to both the source and the sink via directed edges.  
Additionally, each point is connected to its nearest neighboring points through undirected edges.

The algorithm assigns **weights** to the edges in the graph. These weights are of three types:

---

## 1. Smoothness Cost (Between Neighboring Points)

The weight assigned to edges between neighboring cloud points is called the **smooth cost**, calculated as:

\[
\text{smoothCost} = e^{-\left(\frac{d}{\sigma}\right)^2}
\]

where:
- \( d \) is the Euclidean distance between two points,
- \( \sigma \) is a user-defined parameter controlling how rapidly the connection weakens with distance.

**Interpretation:**  
The farther two points are from each other, the higher the likelihood that the edge connecting them will be cut.

---

## 2. Data Cost (Between Points and Source/Sink)

The data cost has two components:

- **Foreground Penalty (Source Edge):**  
  A constant user-defined value assigned to edges between points and the source.  
  This represents the belief that the point belongs to the object (foreground).

- **Background Penalty (Sink Edge):**  
  A value proportional to the point’s distance from the expected object center, calculated as:

\[
\text{backgroundPenalty} = \frac{\text{distanceToCenter}}{\text{radius}}
\]

where:

\[
\text{distanceToCenter} = \sqrt{(x - \text{centerX})^2 + (y - \text{centerY})^2}
\]

with:
- \( (x, y) \) being the horizontal coordinates of the point,
- \( (\text{centerX}, \text{centerY}) \) being the approximate center of the object,
- \( \text{radius} \) being the user-defined expected horizontal extent of the object.

**Interpretation:**  
Points farther from the center are more heavily penalized and thus encouraged to be classified as background.

---

## 3. Graph Cut Optimization

Once the graph is fully constructed and all weights assigned, the algorithm computes the **minimum cut** — the partition that minimizes the total weight of the cut edges.

Based on the result:
- Points connected to the **source** are labeled as **foreground** (belonging to the object).
- Points connected to the **sink** are labeled as **background** (everything else).

---

# Parameters
- **Smooth cost** encourages neighboring points to be classified similarly.
- **Foreground and background penalties** guide the segmentation based on proximity to the object center.
- **Minimum cut** balances smoothness and data penalties to achieve robust segmentation even in noisy or cluttered environments.

---

#  Summary Table

| Component | Description |
|:---|:---|
| Graph nodes | Cloud points + Source + Sink |
| Smoothness cost | \( e^{-\left(\frac{d}{\sigma}\right)^2} \) |
| Foreground penalty | Constant user-defined weight |
| Background penalty | \( \frac{\text{distanceToCenter}}{\text{radius}} \) |
| Optimization | Find the minimum cut for segmentation |

