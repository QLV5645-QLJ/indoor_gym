from multiprocessing import Pool,Process
from contextlib import contextmanager
import time
import numpy as np
import os
from threading import Thread 

from evaluate.load_tsv import TsvLoader
from evaluate.stage_controller import PoseController,VelController


class controller_worker(Thread):
    def __init__(self,trajs,rank=0,type="pose"):
        """
        rank: determine the topic to be listened and published
        type: determine the moving type
        """
        Thread.__init__(self)
        self.rank = rank
        self.trajs = np.copy(trajs)
        self.type = type

    def run(self):
        """
        all thread start learning from here
        """
        if(self.type == "pose"):
            self.move_obj_pose(rank = self.rank, trajs=self.trajs)
        elif (self.type == "vel"):
            self.move_obj_vel(rank = self.rank,trajs = self.trajs)
        else:
            print("no moving object in the thread%d"%self.rank)
        return

    def move_obj_vel(self,rank,trajs):
        # decide robot topic and world fram origin
        cmd_topic = "/robot_%d/cmd_vel"%rank
        odom_topic = "/robot_%d/odom"%rank
        pose_topic = "/robot_%d/cmd_pose"%rank
        crash_topic = 'robot_%d/is_crashed'%rank
        xindex = 2 * rank
        yindex = 2 * rank + 1

        #use velocity controller to move obs
        vc = VelController(cmd_topic,odom_topic,pose_topic,crash_topic)
        for i in range(0,20000,100):
            if(trajs[i,[xindex]] ==0 and trajs[i,[yindex]] ==0):
                # rospy.sleep(1.0)
                # vc.reset()
                continue
            x = (trajs[i,[xindex]]) /1000.0
            y = (trajs[i,[yindex]]) /1000.0
            vc.move_pose([x,y])
            if(x>10.0 or y>10.0):
                print("unexpectical position",x,y,"in rank: ",rank)
        vc.reset()
        return

    def move_obj_pose(self,rank,trajs):
        # move object using pose controller
        pc = PoseController(pose_topic = "/robot_%d/cmd_pose"%rank)    
        xindex = 2 * rank
        yindex = 2 * rank + 1
        for i in range(0,20000,50):
            if(trajs[i,[xindex]] ==0):
                # time.sleep(0.5)
                continue
            x = trajs[i,[xindex]] /1000.0
            y = trajs[i,[yindex]] /1000.0
            pc.move_pose(x,y)
            time.sleep(0.1)


class IndoorGym():
    def __init__(self):
        """
        open a terminal to launch the world
        """
        return

    def make(self,obs_num = 1, exp_id = 1):
        """
        load trajectory data from file,
        """
        # read trajectory of nine people into a array
        script_dir = os.path.abspath(os.getcwd())
        rel_path = '/THOR_dataset/Exp_%d_run_%d.tsv'%(obs_num,exp_id)
        file_path = script_dir+rel_path
        loader = TsvLoader(file_path = file_path)
        trajs = loader.extract_trajectory()
        self.trajs = np.copy(trajs)
        return
    
    def step(self,rank_range = [0],move_type = "pose"):
        """
        make a stage world simulating the human trjectory dataset 
        rank_range: determine the robot topic  needed to listen and publish
        move_type: "pose" or "vel": the type of driving robot
        """
        woker_list = []
        for rank in rank_range:
            temp_worker = controller_worker(trajs=self.trajs,rank=rank,type="pose")
            woker_list.append(temp_worker)
        for worker in woker_list:
            print("thread_%d starts here"%(worker.rank))
            worker.start()
        return

    def reset(self):
        """
        reset the stage world with static object
        """

        return



if __name__ == '__main__':
    pass