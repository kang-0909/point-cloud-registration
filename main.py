import open3d as o3d
import numpy as np
import random
import sys
import copy
from sklearn.decomposition import PCA

volSize = 0.01
disThreshold = 1
lengthThreshold = 0.05
fitThreshold = 2.5 * volSize

def Distance(p1, p2):
    return np.linalg.norm(p1 - p2)

def notTooClose2(p1, p2):
    return np.linalg.norm(p1 - p2) > disThreshold

def notTooClose3(p):
    return notTooClose2(p[0], p[1]) and notTooClose2(p[0], p[2]) and notTooClose2(p[1], p[2])

def farAway(l1, l2):
    ss = np.linalg.norm(l1)
    tt = np.linalg.norm(l2)
    return abs(ss - tt) > lengthThreshold * (ss + tt)

def display_inlier_outlier(pcd, ind):
    inlier_cloud = pcd.select_by_index(ind)
    outlier_cloud = pcd.select_by_index(ind, invert=True)
    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def estimateAvgDis(points):
    sample = random.sample(list(points), 10)
    dis = [Distance(p1, p2) for p1 in sample for p2 in sample if (p1 != p2).all()]
    #print(dis)
    global disThreshold
    disThreshold = np.mean(dis)/2

def prepare(path, color, downSave=False, outlier=False, draw=False, pcaTag=False):
    pcd = o3d.io.read_point_cloud(path)
    pcd.paint_uniform_color(color)
    oldPcd = copy.deepcopy(pcd)
    oldNum = np.asarray(oldPcd.points).shape[0]

    if downSave:
        while True:
            global volSize
            volSize *= 1.1
            pcd = oldPcd.voxel_down_sample(voxel_size=volSize)
            tmp = np.asarray(pcd.points).shape[0]
            if  tmp <= min(10000, oldNum-1):
                break
    else:
        pcd = oldPcd.voxel_down_sample(voxel_size=volSize)
            
    if outlier:
        pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.95)
        if draw:
            display_inlier_outlier(oldPcd, ind)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
    KDT = o3d.geometry.KDTreeFlann(pcd)
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamKNN(knn=200))
    if pcaTag:
        pca = PCA(n_components=pcaTag)
        pca.fit(fpfh.data.transpose())
        fpfh.data = pca.transform(fpfh.data.T).T
    fpfhKDT = o3d.geometry.KDTreeFlann(fpfh)
    global fitThreshold
    fitThreshold = 2.5 * volSize
    return KDT, fpfhKDT, oldPcd, pcd, fpfh.data.T

def calculateTrans(src, tgt):
    assert src.shape == tgt.shape
    src = np.array(src)
    tgt = np.array(tgt)
    num = src.shape[0]
    srcAvg = np.mean(src, axis=0).reshape(1,3)
    tgtAvg = np.mean(tgt, axis=0).reshape(1,3)
    src -= np.tile(srcAvg, (num, 1))
    tgt -= np.tile(tgtAvg, (num, 1))
    H = np.transpose(src) @ tgt
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    T = -R @ srcAvg.T + tgtAvg.T
    return R, T

def ICP(src, tgt):
    print("ICPing...")
    limit = fitThreshold
    retR = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    retT = np.array([[0], [0], [0]])
    trace = []
    for _ in range(400):
        tgtCorr = []
        srcCorr = []
        for point in src:
            k, idx, dis2 = tgtKDT.search_knn_vector_3d(point, knn=1)
            if dis2[0] < (limit)**2:
                srcCorr.append(point)
                tgtCorr.append(tgt[idx[0]])
        trace.append([limit, len(srcCorr)])
        R, T = calculateTrans(np.array(srcCorr), np.array(tgtCorr))
        retR = R @ retR
        retT = R @ retT + T
        src = np.transpose((R @ src.T) + np.tile(T, (1, srcNum)))
        limit = (limit - fitThreshold/1.5) * 0.95 + fitThreshold/1.5
        if len(trace) > 50 and len(set([x[1] for x in trace[-20:]])) == 1:
            break
    print("ICP trace is:", trace[::5])
    return retR, retT

def RANSAC():
    maxCount = 0
    jisuan=0
    j = 0
    print("RANSACing...")
    while True:
        j += 1
        srcCorr = random.sample(range(srcNum), 3)
        if not notTooClose3([srcPoints[x] for x in srcCorr]):
            continue
        tgtCorr = []
        for id in srcCorr:
            k, idx, dis2 = tgtFpfhKDT.search_knn_vector_xd(srcFpfh[id], knn=1)
            tgtCorr.append(idx[0])
            
        if True in [farAway(srcPoints[i[0]] - srcPoints[j[0]], 
                            tgtPoints[i[1]] - tgtPoints[j[1]]) 
                    for i in zip(srcCorr, tgtCorr) 
                    for j in zip(srcCorr, tgtCorr)]:
            continue
        jisuan += 1
        R, T = calculateTrans(np.array([srcPoints[i] for i in srcCorr]), 
                              np.array([tgtPoints[i] for i in tgtCorr]))
        A = np.transpose((R @ srcPoints.T) + np.tile(T, (1, srcNum)))
        
        count = 0
        for point in range(0, srcNum, 1):           
            k, idx, dis2 = tgtKDT.search_hybrid_vector_3d(A[point], 
                                                          radius=fitThreshold, max_nn=1)
            count += k
        if count > maxCount:
            maxCount = count
            bestR, bestT = R, T
        if jisuan > 50 and j > 1000:
            break
        
    print("RANSAC calculated %d times, maximum matches: %d" % (jisuan, maxCount))
    return bestR, bestT

def testTrans():
    rd = np.random.RandomState(888) 
    pcd1=o3d.geometry.PointCloud()
    pcd2=o3d.geometry.PointCloud()
    
    R = np.mat(rd.rand(3,3))
    t = np.mat(rd.rand(3,1))
    U,S,Vt = np.linalg.svd(R)
    R = U*Vt
    if np.linalg.det(R) < 0:
        Vt[2,:]*=-1
        R = U*Vt
    n = 3
    A = np.mat(rd.rand(n,3))
    B = R*A.T + np.tile(t,(1,n))
    B = B.T
    
    ret_R, ret_t = calculateTrans(A,B,3)
    ret_R = np.mat(ret_R)
    ret_t = np.mat(ret_t)
    A2 = (ret_R*A.T)+ np.tile(ret_t,(1,n))
    A2 =A2.T
    
    points1=o3d.utility.Vector3dVector(np.array(B))
    points2=o3d.utility.Vector3dVector(np.array(A2))
    pcd1.points = points1
    pcd2.points = points2
    o3d.visualization.draw_geometries([pcd1, pcd2])

if __name__ == "__main__":
    srcPath = sys.argv[1]
    tgtPath = sys.argv[2]
    savePath = sys.argv[3]
    srcKDT, srcFpfhKDT, oldSrc, src, srcFpfh = prepare(srcPath, [1, 0, 0], downSave=True, outlier=False)
    tgtKDT, tgtFpfhKDT, oldTgt, tgt, tgtFpfh = prepare(tgtPath, [0, 1, 0], outlier=False)
    
    srcPoints = np.array(src.points)
    tgtPoints = np.array(tgt.points)
    srcNum = np.asarray(srcPoints).shape[0]
    tgtNum = np.asarray(tgtPoints).shape[0]
    estimateAvgDis(srcPoints)
    o3d.visualization.draw_geometries([src, tgt])
    
    print("srcNum: %d\ntgtNum: %d" % (srcNum, tgtNum))
    R1, T1 = RANSAC()
    A = np.transpose((R1 @ srcPoints.T) + np.tile(T1, (1, srcNum)))
    A=o3d.utility.Vector3dVector(A)
    src.points = A
    o3d.visualization.draw_geometries([src, tgt])
    
    
    R2, T2 = ICP(np.array(A), tgtPoints)
    R = R2 @ R1
    T = R2 @ T1 + T2
    A = np.array(oldSrc.points)
    A = np.transpose((R @ np.array(A).T) + np.tile(T, (1, A.shape[0])))
    A=o3d.utility.Vector3dVector(A)
    oldSrc.points = A
    o3d.visualization.draw_geometries([oldSrc, oldTgt])
    o3d.io.write_point_cloud("results/" + savePath, oldSrc + oldTgt)
    print('\nrotation:\n', R)
    print('transition:\n', T)
    