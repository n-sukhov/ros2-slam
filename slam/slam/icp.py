#!/usr/bin/env python3
import numpy as np
from numpy.linalg import svd, det
from sklearn.neighbors import NearestNeighbors

class ICP():
    def __init__(self, max_iterations=100, tolerance=0.0001):
        self.max_iterations=max_iterations
        self.tolerance=tolerance
        
    def __calculate_transform_matrix(self, A, B):
        n = A.shape[1]
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        A_centered = A - centroid_A
        B_centered = B - centroid_B

        H = A_centered.T @ B_centered
        U, S, VT = svd(H)
        R = VT.T @ U.T
        
        if det(R) < 0:
            VT[n-1,:] *= -1
            R = VT.T @ U.T
        
        t = centroid_B.T - R @ centroid_A.T
        T = np.identity(n + 1)
        T[:n, :n] = R
        T[:n, -1] = t.ravel()

        return T

    def __find_nearest_neighbor(self, src, dst):
        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()

    def get_T(self, src, dst):
        n = src.shape[1]
        transformed_src_hom = np.hstack((src, np.ones((src.shape[0], 1)))).T
        prev_error = float('inf')
        T_total = np.identity(n + 1)
    
        for i in range(1, self.max_iterations):
            current_src = transformed_src_hom[:n, :].T 
            distances, indices = self.__find_nearest_neighbor(current_src, dst)
            T_current = self.__calculate_transform_matrix(current_src, dst[indices])
            transformed_src_hom = T_current @ transformed_src_hom
            mean_error = np.mean(distances)
            
            if np.abs(prev_error - mean_error) < self.tolerance:
                break
                
            prev_error = mean_error
            T_total = T_current @ T_total
    
        T_result = self.__calculate_transform_matrix(src, transformed_src_hom[:n].T)
    
        return T_result, prev_error