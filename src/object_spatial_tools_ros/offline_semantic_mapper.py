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
    
    def __init__(self, raw_map_path, objects_types = [], dist_cut = 0):
        
        self.dist_cut = dist_cut
        
        self.RAW_MAP = self.load_raw_map(raw_map_path)
        print(f'Loaded file {raw_map_path}')
        for obj, data in self.RAW_MAP.items():            
            self.RAW_MAP[obj] = data[np.where(data[:,OfflineMapper.D] > dist_cut)]
            
        
        if len(objects_types) == 0:
            self.objects_types = list(self.RAW_MAP.keys())
        else:
            self.objects_types = [obj for obj in objects_types if obj in self.RAW_MAP]
        print(f'Taken objects {self.objects_types}')
        
            
    def calc_outliers(self, n_neighbors = 2):
        clf = LocalOutlierFactor(n_neighbors = n_neighbors)
        self.outliers = {}
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types:                
                labels = np.array(clf.fit_predict(data[:,:2]))
                #print(labels)
                labels[labels == -1] = 0
                self.outliers[obj] = np.array(labels, dtype = 'bool')
        print(self.outliers)
        
    def apply_outliers(self):
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types:     
                self.RAW_MAP[obj] = data[self.outliers[obj], :]
        
    def calc_clusters(self, n_clusters):
        clustering = SpectralClustering(n_clusters=n_clusters, assign_labels='discretize', random_state=0)
        
        self.clusters = {}
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types:
                clustering.fit(data[:,:3])
                self.clusters[obj] = clustering.labels_
            
                
    def plot_with_clusters(self):
        for obj, data in self.RAW_MAP.items():
            if obj in self.objects_types:
                n_clusters = np.unique(self.clusters[obj]).shape[0]
                for cl in range(n_clusters):
                    plt.plot(data[self.clusters[obj] == cl,0], data[self.clusters[obj] == cl,1], '.', label = f'{obj}#{cl}')
                    
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
                plt.plot(data[self.outliers[obj],0], data[self.outliers[obj],1], '.', label = obj)
                plt.plot(data[np.logical_not(self.outliers[obj]),0], data[np.logical_not(self.outliers[obj]),1], 'x', label = f'{obj} outliers')                
        
        plt.legend()
        plt.gca().axis('equal')
        plt.grid()
        plt.title('Raw map with outliers')
        plt.show()
    
    
    

if __name__ == '__main__':
    
    om = OfflineMapper('/home/anton/scene_testing_ws/src/ritrover-scene-localization/config/raw_poses.yaml', ['door'], 6)
        
        
    om.calc_outliers(10)
    om.plot_raw_with_outliers()
    om.apply_outliers()
    #om.plot_raw_map()
    om.calc_clusters(9)
    om.plot_with_clusters()
    
    
    
    
        
        

