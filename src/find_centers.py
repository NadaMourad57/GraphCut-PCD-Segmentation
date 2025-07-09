# import open3d as o3d
# import numpy as np

# def geometric_median(X, eps=1e-5, max_iter=1000):
#     # y = approximate_centers[0]
#     y= np.mean(X, axis=0)
#     for i in range(max_iter):
#         D = np.linalg.norm(X - y, axis=1)
#         nonzero = D > eps
#         if not np.any(nonzero):
#             return y
#         W = 1  /D[nonzero]
#         T = X[nonzero]
#         y_new = (W[:, None] * T).sum(axis=0) / W.sum()
#         if np.linalg.norm(y - y_new) < eps:
#             return y_new
#         y = y_new
#     return y





# if __name__ == "__main__":


#     # approximate_centers=[
#     # [0.04197685726428131, -0.10707418202773965, 0.4863020500324206],
#     # [0.05936154565764844, -0.15338471096914613, 0.4684879290055432],
#     # [0.07476264655662287, -0.17911226413076417, 0.40912986617474834],
#     # [0.07557618289699268, -0.17764848487121437, 0.35450979571467334],
#     # [0.055017018960903386, -0.18004765448747073, 0.31198373704508897]
#     # ]


#     approximate_centers=[
#     [0.027040696692439622, -0.007762121469941318, 0.46109863608452334],
#     [0.07736941750663692, -0.005798843093527724, 0.4792014970604352],
#     [0.12285113903136376, -0.009990935471402462, 0.4852797918847246],
#     [0.1771804392789689, -0.01783069133646544, 0.4474605478557555],
#     [0.1929358184498834, -0.00645824278903111, 0.39481420482984536]

#     ]


#     for i, center in enumerate(approximate_centers):

#         pcd = o3d.io.read_point_cloud(f"ExtractedCutters/06_02_2024_08_18/filtered_cloud_{i}.pcd")
#         pcd.paint_uniform_color([0.5, 0.5, 0.5])  # Gray color
#         points = np.asarray(pcd.points)

#         median = geometric_median(points)
#         bbox = pcd.get_axis_aligned_bounding_box()  # or get_oriented_bounding_box()

#         median_bbox=bbox.get_center()



#         # add aproximate center in yellow
#         approx_med = o3d.geometry.TriangleMesh.create_sphere(radius=0.001)
#         approx_med.translate(approximate_centers[i])
#         approx_med.paint_uniform_color([0, 0, 1])  # blue


#         bbox_med = o3d.geometry.TriangleMesh.create_sphere(radius=0.001)
#         bbox_med.translate(median_bbox)
#         bbox_med.paint_uniform_color([0,1, 0])  # green



#         geometric_med = o3d.geometry.TriangleMesh.create_sphere(radius=0.001)
#         geometric_med.translate(median)
#         geometric_med.paint_uniform_color([1, 0, 0])  # Red center point



#         o3d.visualization.draw_geometries([pcd, geometric_med, approx_med, bbox_med, bbox])



import cvxpy as cp
import numpy as np

def geometric_median_cvxpy(X):
    y = cp.Variable(3)
    objective = cp.Minimize(cp.sum(cp.norm(X - y, axis=1)))
    problem = cp.Problem(objective)
    problem.solve()
    return y.value
