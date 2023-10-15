# -*- coding: utf-8 -*-

# A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases with Noise
# Noh Kyungha 
# 2023-06-21

import numpy as np
import math

UNCLASSIFIED = False
NOISE = None


def _dist(x1,y1,x2,y2): # distance of two points
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def _is_neighbor(x1,y1,x2,y2,eps): # check two point is neighbor
    return _dist(x1,y1,x2,y2) < eps

def _region_query(X,Y, point_id, eps): # return neighbor points (X,Y: x,y componets of pointcloud)
    n_points = len(X)
    seeds = []
    for i in range(0, n_points):
        if _is_neighbor(X[point_id],Y[point_id],X[i],Y[i],eps):
            seeds.append(i)
    
    return seeds

def _expand_cluster(X,Y,classifications,point_id, cluster_id,eps, min_points):
    seeds = _region_query(X,Y, point_id, eps)
    if len(seeds) < min_points:
        classifications[point_id] = NOISE
        return False
    else:
        classifications[point_id] = cluster_id
        for seed_id in seeds:
            classifications[seed_id] = cluster_id
        while len(seeds) > 0:
            current_point = seeds[0]
            results = _region_query(X,Y, current_point, eps)
            if len(results) >= min_points:
                for i in range(0, len(results)):
                    result_point = results[i]
                    if classifications[result_point] == UNCLASSIFIED or \
                       classifications[result_point] == NOISE:
                        if classifications[result_point] == UNCLASSIFIED:
                            seeds.append(result_point)
                        classifications[result_point] = cluster_id
            seeds = seeds[1:]
        return True

def dbscan(X,Y,eps,min_points):
    cluster_id =1
    n_points = len(X)
    classifications = [UNCLASSIFIED] * n_points

    for point_id in range(0, n_points):
        pX,pY = X[point_id],Y[point_id]
        if classifications[point_id] == UNCLASSIFIED:
            if _expand_cluster(X,Y, classifications, point_id, cluster_id, eps, min_points):
                cluster_id = cluster_id + 1
    return classifications,cluster_id # return cluster classification and number of clusters

def center_of_cluster(X,Y,lidar_array,classifications,cluster_num):
    n_points = len(X)
    cluster_x = np.zeros(cluster_num)
    cluster_y = np.zeros(cluster_num)
    
    # 클러스터의 평균점 구함
    for i in range(cluster_num):   # for each clusters
        n_cluster_element = 0
        for point_idx in range(0, n_points):
            if classifications[point_idx] == i:
                cluster_x[i] += X[point_idx]
                cluster_y[i] += Y[point_idx]
                n_cluster_element += 1
        cluster_x[i] = cluster_x[i] / n_cluster_element
        cluster_y[i] = cluster_y[i] / n_cluster_element

    # 가까운 거리 순서로 정렬
    cluster_dist = cluster_x*cluster_x + cluster_y*cluster_y
    order_cluster = cluster_dist.argsort()

    sortedX = cluster_x[order_cluster]
    sortedY = cluster_y[order_cluster]

    return sortedX,sortedY


    
            

# want : 거리순으로 클러스터의 평균 x,y좌표 반환
if __name__ == "__main__":
    ang = [1]*10
    print(ang)