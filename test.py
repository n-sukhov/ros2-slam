def find_nearest_neighbor(start_point_cloud, goal_point_cloud):
    distances = []
    indices = []

    for i, point in enumerate(start_point_cloud):
        min_dist = float('inf')
        best_idx = -1

        for j, goal_point in enumerate(goal_point_cloud):
            dist = np.sqrt(np.sum((point - goal_point)**2))
            if dist < min_dist:
                min_dist = dist
                best_idx = j

        distances.append(min_dist)
        indices.append(best_idx)

    return np.array(distances), np.array(indices)