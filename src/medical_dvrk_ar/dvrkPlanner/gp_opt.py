#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
from sklearn import gaussian_process
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from scipy.stats import multivariate_normal, norm
from scipy.interpolate import griddata
import PIL
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge, CvBridgeError


from dvrk import psm
import math
import PyKDL
import argparse
import os.path
from util import make_PyKDL_Frame, calculate_stiffness, nearest_point
from robot_motion import ControlServer_palpation

np.random.seed(1)

class aquisition_algorithm(object):
    """Abstract class for the different algorithms"""
    def __init__(self, estimated_map):
        self.estimated_map = estimated_map

    def aquisitionFuncitons(self):
        raise Exception("NotImplementedException")

class EI(aquisition_algorithm):
        """Expected improvement class"""
        def __init__(self, estimated_map, stiffnessCollected):
            super(EI, self).__init__(estimated_map)
            self.stiffnessCollected=stiffnessCollected

        def aquisitionFunciton(self):
            eps = 0.1
            ymu = self.estimated_map['mean']
            ys2 = self.estimated_map['variance']

            minStiffnessCollected = np.min(self.stiffnessCollected)

            ys=np.atleast_2d(np.sqrt(ys2)).T;
            yEI=np.max(self.stiffnessCollected) - minStiffnessCollected

            ymuMax=np.max(ymu)
            ymu=ymu/ymuMax
            yEI=yEI/ymuMax
            ind_zero = (ys<=0.01)
            aquisitionFunciton = (ymu-yEI-eps)*norm.cdf((ymu-yEI-eps)/ys) + ys*norm.pdf((ymu-yEI-eps)/ys);
            aquisitionFunciton[ind_zero]=0;
            aquisitionFunciton/=np.sum(aquisitionFunciton)
            return aquisitionFunciton

class UCB(aquisition_algorithm):
    """Upper confidence bound class"""
    def __init__(self, estimated_map,_):
        super(UCB, self).__init__(estimated_map)

    def aquisitionFunciton(self):
        beta=4.35
        ymu = self.estimated_map['mean']
        ys2 = self.estimated_map['variance']
        ymu=ymu/np.max(ymu)

        ys=np.atleast_2d(np.sqrt(ys2)).T;
        aquisitionFunciton = ymu + beta*ys;
        aquisitionFunciton/=np.sum(aquisitionFunciton)
        return aquisitionFunciton

class LSE(aquisition_algorithm):
    """docstring for LSE"""
    def __init__(self, estimated_map,_):
        super(LSE, self).__init__(estimated_map)
        self.C = None
        self.a = None #ambiguity
        self.h = .8
        self.beta=1.35

    def aquisitionFunciton(self):
        ymu = self.estimated_map['mean']
        ymuMax=np.max(ymu)
        if ymuMax != 0:
            ymu=ymu/ymuMax

        ys2 = self.estimated_map['variance']
        ys=np.atleast_2d(np.sqrt(ys2)).T;

        Q_min = ymu - self.beta*ys;
        Q_max = ymu + self.beta*ys;
       
        if self.C is None:
            self.C = {'min':Q_min,'max':Q_max}
        else:
            self.C['min']=np.max(np.array([Q_min,self.C['min']]),axis = 0)
            self.C['max']=np.min(np.array([Q_max,self.C['max']]),axis = 0)

        temp1=self.C['max']-self.h*np.ones(self.C['max'].shape)
        temp2=self.h*np.ones(self.C['max'].shape)-self.C['min']
        self.a=np.min(np.array([temp1.T[0],temp2.T[0]]),axis=0)
        return self.a

class gpr_palpation():
    def __init__(self, data, frequency, amplitude, dest_folder, algorithm_name, visualize=True,  simulation=True, wait_for_searching_signal = True):
        self.searching = not wait_for_searching_signal
        self.simulation = simulation
        self.domain = {'L1':100, 'L2':100}
        self.grid = self.generateGrid()
        #print(self.grid.shape)
        self.groundTruth = self.generateStiffnessMap()
        self.gp = self.gp_init()
       
        self.estimated_map={'mean':None,'variance':None}
        self.probedPoints=[] # saves all the probed points so far
        self.stiffnessCollected=[] # saves all the probed stiffnesses
       
        self.algorithm_class = self.chooseAlgorithm(algorithm_name)

        # ROS
        self.pub = rospy.Publisher('/stiffness_map', Image, queue_size=10)       

        self.rate = rospy.Rate(10) #1000
        self.visualize = visualize
    
        # self.sim_start_time is the start time (ros time) of the simulation
        self.sim_start_time = None
        # self.freq is the estimated frequency of the liver
        self.freq = frequency
        # self.amp is the estimated amplitude of liver
        self.amp = amplitude
        # self.data is the filtered data 
        self.data = data
        # locations to evaluate with gaussian process (instead of using grid)
        self.locations = data[:,0:2]
        self.x_span  = self.locations[:,0].max() - self.locations[:,0].min()
        self.y_span = self.locations[:,1].max() - self.locations[:,1].min()
        # self.server is the motion server for the robot
        self.server = ControlServer_palpation(amplitude, frequency, self.data)
        # self.number_of_data is the total number of data points
        self.number_of_data = data.shape[0]
        self.ind = np.random.randint(0,self.number_of_data)
        # self.data_not_probed record whether a point has been probed, not probed with 1, probed with 0
        self.data_not_probed = np.ones(data.shape[0])
        # self.output_nparray is the output file of the palpation
        self.output_nparray = [] 
        print('number of data', self.number_of_data)
        self.dest_folder = dest_folder
        self.local_poke_times = 0
       

    def chooseAlgorithm(self, algorithm_name):
        if algorithm_name == 'EI':
            return EI(self.estimated_map,self.stiffnessCollected)
        if algorithm_name == 'UCB':
            return UCB(self.estimated_map,self.stiffnessCollected)
        if algorithm_name == 'LSE':
            return LSE(self.estimated_map,None)

    def gp_init(self):
        kernel = C(1.0, (1e-3, 1e3)) * RBF(1, (8, 20))
        gp = gaussian_process.GaussianProcessRegressor(kernel=kernel, optimizer='fmin_l_bfgs_b', n_restarts_optimizer=9)
        return gp

    def generateGrid(self,res=1):
        x = np.linspace(0, self.domain['L1'], self.domain['L1']/res)
        y = np.linspace(0, self.domain['L2'], self.domain['L2']/res)
        Xg,Yg = np.meshgrid(x,y)
        grid = np.array([Xg.flatten(), Yg.flatten()]).T
        return grid

    def generateStiffnessMap(self):
        '''
        generates random ground truth(stiffness map) of an organ for simulation purposes
        '''
        m1=[20.0,20.0]
        s1=60.0*np.identity(2)
        m2=[30.0,60.0]
        s2=60.0*np.identity(2)
        m3=[60.0,60.0]
        s3=150.0*np.identity(2)
        m4=[70.0,20.0]
        s4=60*np.identity(2);

        grid = self.grid

        mvn1 = multivariate_normal(m1,s1)
        G1 = mvn1.pdf(grid)
        mvn2 = multivariate_normal(m2,s2)
        G2 = mvn2.pdf(grid)
        mvn3 = multivariate_normal(m3,s3)
        G3 = mvn3.pdf(grid)
        mvn4 = multivariate_normal(m4,s4)
        G4 = mvn4.pdf(grid)

        G=G1+G2+2*G3+G4
        # G=np.max(G,0); #crop below 0
        G[G<0.0]=0.0
        G=1000*G + 9000; #normalize
        G=G/np.max(G)
        # print(np.max(G))
        # print(np.min(G))
        return G

    def visualize_map(self, title, figure, map=None, probed_points=None):
        if map is None:
            dense_grid=self.generateGrid(res=1)
            ymu = self.gp.predict(self.grid, return_std=False)
            ymu[ymu<0]=0
            map = ymu

        map=map.reshape(self.domain['L1'],self.domain['L2'])
        normalized_map = map/np.max(map)

        im = PIL.Image.fromarray(np.uint8(cm.hot(normalized_map)*255))
        cv_im=np.array(im)

        msg_frame = CvBridge().cv2_to_imgmsg(cv_im,'rgba8')
        self.pub.publish(msg_frame)
       
        if self.visualize:
            plt.figure(figure)
            plt.clf()
            fig=plt.imshow(normalized_map, origin='lower',cmap=cm.hot)
            plt.title(title)
            plt.colorbar()
            if not (probed_points is None):
                plt.scatter(probed_points[:,0],probed_points[:,1])
            # plt.xlim((0,100))
            # plt.ylim((0,100))
            # plt.tight_layout()
            # plt.show()
            #filename = str(len(probed_points)).zfill(4)
            filename = "test"
            plt.savefig(filename + '.png')
            #np.savetxt(filename + '.txt', self.estimated_map['mean'])
            plt.pause(0.01)

        return msg_frame
       
    def evaluateStiffness(self, X_query):
        return griddata( self.grid, self.groundTruth, X_query)

    def probe(self, x_probed, point_index):
        self.probedPoints.append(x_probed.tolist())
        if self.simulation:
            #evaluate the stiffness at that point
            print("Poking on simulated data")
            stiffness = self.evaluateStiffness(X_query=x_probed)
            print("stiffness of new point:", stiffness)

        else: 
            ##### motion compensation
            self.data_not_probed[point_index] = 0
            dest = make_PyKDL_Frame(self.data[point_index])
            self.server.move(dest, self.server.maxForce)
            # append current pose data
            currentPose = self.server.robot.get_current_position() #PyKDLFrame
            translation = [currentPose.p[0],currentPose.p[1],currentPose.p[2]]
            #rotation = currentPose.M.GetQuaternion()
            run_time = rospy.Time.now().to_sec()
            offset_z = self.amp * math.sin(self.freq * run_time)
            translation[2] -= offset_z
            which_tumor, euclid_norm, stiffness, tumor_or_not = calculate_stiffness(translation, self.dest_folder)[:]
            print("stiffness of new point:", stiffness)
            point_data = (translation[0],translation[1],translation[2], which_tumor, euclid_norm, stiffness, tumor_or_not)
            self.output_nparray.append(point_data)
    
            if stiffness == None:
                rospy.logwarn("Got invalid stiffness. Ignoring value")
                self.probedPoints = self.probedPoints[:len(self.probedPoints)-1]
                return
            stiffness = np.array([stiffness])

        self.stiffnessCollected.append(stiffness.tolist())

        #print("All stiffness collected:",self.stiffnessCollected)
        probedPoints_array = np.asarray(self.probedPoints)
        stiffnessCollected_array = np.asarray(self.stiffnessCollected)
        # minStiffness = np.min(stiffnessCollected_array)
        # stiffnessCollected_array = stiffnessCollected_array - minStiffness
        self.gp.fit(probedPoints_array, stiffnessCollected_array)
        self.estimated_map['mean'], self.estimated_map['variance'] = self.gp.predict(self.locations, return_std=True)
        self.estimated_map['mean'] -= np.min(stiffnessCollected_array)
        self.estimated_map['mean'][self.estimated_map['mean']<0] = 0
        #print("mean", self.estimated_map['mean'])
        #print("mean", self.estimated_map['mean'].shape)
        #print("variance", self.estimated_map['variance'].shape)

        # shows the animation of the stiffness estimation
        #self.visualize_map(title='Estimated map', figure=2, map=self.estimated_map['mean'], probed_points=probedPoints_array)

    def nextBestPoint(self, alg):
        '''
        Takes in a funciton to estimate next best point accordingly. For now it supports 'EI' and 'UCB'
        '''
        self.aquisitionFunciton = alg.aquisitionFunciton()
        indices = np.array(sorted(range(len(self.aquisitionFunciton)),reverse=True, key=lambda x: self.aquisitionFunciton[x]))
        indices = indices[np.array(np.nonzero(self.data_not_probed)[0],dtype=int)]
        thresh = 100
        dis = np.linalg.norm(self.locations[indices[:thresh],:] - self.locations[self.ind,:],axis=-1)

        if np.min(dis) < min(self.x_span, self.y_span) / 5:
            ind = indices[np.argmin(dis)]
            self.local_poke_times += 1
            if self.local_poke_times == 20:
                self.local_poke_times = 0 
                print("............Start more exploration............")
                ind = indices[np.random.randint(0,len(indices))]
        else:
            dis = np.linalg.norm(self.locations - self.locations[self.ind,:],axis=-1)
            ind =indices[np.array(np.nonzero(dis < (min(self.x_span, self.y_span) / 5))[0])[0]]
            self.local_poke_times = 0

        """
        i=0
        ind = indices[i]
        while i <= self.number_of_data and (not self.data_not_probed[ind] or np.abs(self.ind - ind) > 400):
            i=i+1
            ind = indices[i]
        """
        return ind
   
    def autoPalpation(self, num_of_probes=-1):
        self.server.robot.home()

        # record the start time of the simulation
        self.sim_start_time = rospy.Time.now().to_sec()
        self.robot_pose = self.server.robot.get_current_position()

        if num_of_probes == -1:
            print(self.searching)
            rate = rospy.Rate(100)
            i = 0
            while not rospy.is_shutdown():
                if not self.searching:
                    i = 0
                    rate.sleep()
                    continue
                i = i + 1
                print('Probing point: '+str(i))
                x_probe = self.grid[self.ind,:]
                self.probe(x_probe,self.ind)
                self.ind = self.nextBestPoint(self.algorithm_class) #Here change which algorithm you want
        else:
            
            num_burn_in = 20
            print("............Phase 1: Random exploration............")
            for i in range(num_burn_in):
                self.ind = np.random.randint(0,self.number_of_data)
                while not self.data_not_probed[self.ind]:
                    self.ind = np.random.randint(0,self.number_of_data)
                x_probe = self.locations[self.ind]
                print("Probing index:", self.ind)
                self.probe(x_probe, self.ind)

            print("............Phase 2: Fine grained exploration............")
            for i in range(num_of_probes-num_burn_in):
                file_path = self.dest_folder + "/"
                file_name_time = "palpation_time_" + str(i+num_burn_in) + "pokings.npy"
                file_name = "palpation_result_" + str(i+num_burn_in) + "pokings.npy"
                if (i + num_burn_in) % 50 == 0:
                    print("............."+str(i+num_burn_in) + "pokes finished!.............")
                    np.save(file_path+file_name_time, time.time()-start_time)
                    np.save(file_path+file_name, np.array(self.output_nparray))

                x_probe = self.locations[self.ind]
                print("Probing index:", self.ind)
                self.probe(x_probe, self.ind)
                self.ind = self.nextBestPoint(self.algorithm_class) #Here change which algorithm you want

                if i == num_of_probes-num_burn_in-1:
                    np.save(file_path+file_name_time, time.time()-start_time)
                    np.save(file_path+file_name, np.array(self.output_nparray))
                    break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='read the point cloud data for palpation')
    parser.add_argument('--path',help='the path to the palpation npy file')
    parser.add_argument('--dest', help = 'which folder to store the file')
    parse.add_argument('--num_pokes', help ='how many number of pokes')
    args = parser.parse_args()

    file_path = args.path
    dest_folder = args.dest
    num_pokes = args.num_pokes
    data = np.load(file_path)
    frequency = 0.5 #0.5
    amplitude = 0.02  #0.02
    np.random.seed(int(time.time())) 

    global start_time
    start_time = time.time()
    rospy.init_node('gpr_python', anonymous=True)
    gpr = gpr_palpation(data, frequency, amplitude, dest_folder, algorithm_name='UCB', visualize=False, simulation=False, wait_for_searching_signal = True) # 'LSE', 'EI', 'UCB'

    # visualize ground truth
    #gpr.visualize_map(map=gpr.groundTruth,title='Ground Truth', figure=1)
    gpr.autoPalpation(num_pokes)

    
# plt.show()