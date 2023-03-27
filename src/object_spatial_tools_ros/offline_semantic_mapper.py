#!/usr/bin/env python3
# coding: utf-8

import yaml
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.neighbors import LocalOutlierFactor
from sklearn.cluster import SpectralClustering

    
class OfflineMapper(object):
    X = 0
    Y = 1
    Z = 2
    TS = 3
    D = 4
    YAW = 5  
    CAM = 6
    
    N_CL = 0
    N_OL = 1
    
    def __init__(self, raw_map_path, objects_types = {}, dist_cut = 0, ts_cut = 0, clusters_sizes = []):
                    
        
        self.RAW_MAP = self.load_raw_map(raw_map_path)
        print(f'Loaded file {raw_map_path}')
        # cut more than dist_cut
        self.dist_cut = dist_cut
        for obj, data in self.RAW_MAP.items():            
            self.RAW_MAP[obj] = data[np.where(data[:,OfflineMapper.D] < dist_cut)]
        # cut less than ts_cut
        self.ts_cut = ts_cut + data[0, OfflineMapper.TS]
        for obj, data in self.RAW_MAP.items():                        
            self.RAW_MAP[obj] = data[np.where(data[:,OfflineMapper.TS] > self.ts_cut)]                            
                    
        if len(objects_types) == 0:
            print(f'Specify objects and its cluster sizes')
            return
        else:
            self.objects_types = {obj: num for obj, num in objects_types.items() if obj in self.RAW_MAP}
        print(f'Taken objects {self.objects_types}')
        
            
    def calc_outliers(self):        
        self.outliers = {}
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types.keys():                
                if self.objects_types[obj][OfflineMapper.N_OL] == 0:
                    continue
                clf = LocalOutlierFactor(n_neighbors = self.objects_types[obj][OfflineMapper.N_OL])
                labels = np.array(clf.fit_predict(data[:,:2]))
                #print(labels)
                labels[labels == -1] = 0
                self.outliers[obj] = np.array(labels, dtype = 'bool')
        #print(self.outliers)
        
    def apply_outliers(self):
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types.keys():     
                if self.objects_types[obj][OfflineMapper.N_OL] == 0:
                    continue
                self.RAW_MAP[obj] = data[self.outliers[obj], :]
        
    def calc_clusters(self):                        
        
        self.clusters = {}
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types.keys():                                                
                
                clustering = SpectralClustering(n_clusters=self.objects_types[obj][OfflineMapper.N_CL], assign_labels='discretize', random_state=0, affinity='rbf', gamma = 1)
                
                clustering.fit(data[:,:3])
                self.clusters[obj] = clustering.labels_                
            
                
    def plot_with_clusters(self, raw = True):
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types.keys():
                n_clusters = np.unique(self.clusters[obj]).shape[0]
                if n_clusters > 10:
                    palette = sns.color_palette("tab20", 20)
                else:
                    palette = sns.color_palette("tab10", 10)
                for cl in range(n_clusters):
                    cluster = data[self.clusters[obj] == cl,:]
                    color = palette[cl % len(palette)]
                    if raw:
                        plt.plot(cluster[:,0], cluster[:,1], '.', label = f'{obj}#{cl}', color = color, alpha = 0.1)
                    plt.plot(np.mean(cluster[:,0]), np.mean(cluster[:,1]), 'o', label = f'{obj}centroid #{cl}', color = color)
                    
        plt.legend()
        plt.gca().axis('equal')
        plt.grid()
        plt.title('Clustered')
        plt.show()
                    
    def load_raw_map(self, path):
        with open(path, 'r') as infile:
            return yaml.load(infile, Loader = yaml.Loader)
    
    def plot_raw_map(self):        
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types:
                plt.plot(data[:,0], data[:,1], '.', label = obj)
            
        plt.legend()
        plt.gca().axis('equal')
        plt.grid()
        plt.title('Raw map')
        plt.show()
                
        
    def plot_raw_with_outliers(self):
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types:
                if self.objects_types[obj][OfflineMapper.N_OL] == 0:
                    plt.plot(data[:,0], data[:,1], '.', label = obj)
                else:
                    plt.plot(data[self.outliers[obj],0], data[self.outliers[obj],1], '.', label = obj)
                    plt.plot(data[np.logical_not(self.outliers[obj]),0], data[np.logical_not(self.outliers[obj]),1], 'x', label = f'{obj} outliers')                
        
        plt.legend()
        plt.gca().axis('equal')
        plt.grid()
        plt.title('Raw map with outliers')
        plt.show()
    
    
    

if __name__ == '__main__':
    
    
    
    om = OfflineMapper('/home/anton/scene_testing_ws/src/ritrover-scene-localization/config/raw_poses.yaml', {'door': (10, 10), 'poster': (18, 16), 'table': (8, 10), 'cupboard': (8, 10), 'stairs': (2, 5), 'chair': (4, 10)}, 6, 2)
    
    # done things
    # 'door': (10, 10)
    # 'poster': (18, 16)
    # 'table': (8, 10)
    # 'cupboard': (8, 10)
    # 'stairs': (2, 5)
    # 'chair': (4, 10)
    
        
    om.calc_outliers()
    om.plot_raw_with_outliers()
    om.apply_outliers()
    #om.plot_raw_map()
    om.calc_clusters()
    om.plot_with_clusters(raw = False)
    
    
    
    
        
        

