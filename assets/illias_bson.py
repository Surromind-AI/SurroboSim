import io
import bson                     # this is installed with the pymongo package
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
import cv2
import shutil
import matplotlib.pyplot as plt
import enum
import os



class load_bson():
    def __init__(self):
        # # Bson File Path
        self._bson_path = None
        self._action_path = None

        # Bson Load Data
        self.episode_data = None
        self.class_data = None

        # data
        self.object_name = None
        self.all_data = None


    def _load_bson(self, data_path):
        with open(data_path, 'rb') as f:
            data = bson.decode_all(f.read())
        return data

    def _load_class(self, action_data):
        # action_data = self._load_bson(self._action_path)
        action_data = action_data[0]['individuals']
        
        #딕셔너리 선언
        action_dict = dict()
        for tmp_data in action_data:
            action_dict[tmp_data['id']] = tmp_data['class']
        return action_dict

    def timestep_data_check(self, time_step_data):
        # get time step data and return object position in time

        # initialize dict 
        tmp_dict = dict()
        tmp_dict['timestamp']    = time_step_data['timestamp']
        tmp_dict['object_dummy'] = []
        tmp_dict['hand']         = []
        tmp_dict['hand_L']       = []
        tmp_dict['hand_R']       = []
        tmp_dict['MountingBar']  = []
        tmp_dict['bones']        = []
        tmp_dict['etc']          = []

        
        tmp_dict['hand_R_bones'] = []
        tmp_dict['hand_L_bones'] = []



        # save individuals
        individuals_object_dummy = ['HangingDummy', 'HangingDummyWide', 'HangingDummyLong']
        individuals_hand         = ['GenesisLeftHand', 'GenesisRightHand', 'Palm', 'Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        individuals_MountingBar  = ['DMShelfMountingBar']
        individuals_bones        = ['VirtualBoneIndividual', 'BoneRevoluteJoint', 'BoneRevoluteJoint']

        individuals_hand_R_bones = ['GenesisRightHand']
        individuals_hand_L_bones = ['GenesisLeftHand']
        
        for step_data in time_step_data['individuals']:

            ob_name = self.object_name[step_data['id']]
            step_data['id'] = ob_name

            if step_data['id'] in individuals_object_dummy:
                tmp_dict['object_dummy'].append(step_data)
            elif step_data['id'] in individuals_hand:
                tmp_dict['hand'].append(step_data)
            elif step_data['id'] in individuals_MountingBar:
                tmp_dict['MountingBar'].append(step_data)
            elif step_data['id'] in individuals_bones:
                tmp_dict['bones'].append(step_data)
            else:
                tmp_dict['etc'].append(step_data)
        
        for step_data in time_step_data['skel_individuals']:

            ob_name = self.object_name[step_data['id']]
            step_data['id'] = ob_name
            if step_data['id'] in individuals_hand_R_bones:
                tmp_dict['hand_R_bones'].append(step_data)
            elif step_data['id'] in individuals_hand_L_bones:
                tmp_dict['hand_L_bones'].append(step_data)


        tmp_hand_RL = None
        hand_skel = ['Palm', 'Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        for hand_data in tmp_dict['hand']:
            if hand_data['id'] == 'GenesisLeftHand':
                tmp_hand_RL = 'L'
                tmp_dict['hand_L'].append(hand_data)
            elif hand_data['id'] == 'GenesisRightHand':
                tmp_hand_RL = 'R'
                tmp_dict['hand_R'].append(hand_data)
            
            if hand_data['id'] in hand_skel:
                if tmp_hand_RL == 'L':
                    tmp_dict['hand_L'].append(hand_data)
                elif tmp_hand_RL == 'R':
                    tmp_dict['hand_R'].append(hand_data)
                else:
                    print('something wrong !!')
                    tmp_dict['hand_L'] = []
                    tmp_dict['hand_R'] = []
                    break
        return tmp_dict

    def load_episode(self, datas):
        # episode data를 로드해서 시간 순으로 표시
        time_data_list = []
        for idx, data in tqdm(enumerate(datas)):
            time_data_list.append(self.timestep_data_check(data))
        return time_data_list

    def get_timestep_data(self, bson_path, action_path):
        # make action dict
        self._bson_path = bson_path
        self._action_path = action_path
        self.class_data   = self._load_bson(self._action_path)
        self.object_name  = self._load_class(self.class_data)
        self.episode_data = self._load_bson(self._bson_path)
        return self.load_episode(self.episode_data)


class save_plt_video():
    def __init__(self):
        self.color = self.set_color()
        self.txt_path = None

    def set_color(self):
        tmp_dic = dict()
        tmp_dic['object_dummy'] = 'black'
        tmp_dic['hand_L']       = 'chartreuse'
        tmp_dic['hand_R']       = 'aqua'
        tmp_dic['MountingBar']  = 'navy'
        return tmp_dic
    
    def figure_to_array(self, fig):
        """
        plt.figure를 RGBA로 변환(layer가 4개)
        shape: height, width, layer
        """
        fig.canvas.draw()

        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        return image

    def get_list_pose(self, pose_list, object_title, ax):
        tmp_list = []
        for pose in pose_list[object_title]:
            ax.scatter(pose['pose'][0], pose['pose'][1], pose['pose'][2], s=10, c = self.color[object_title])
    
    def get_list_6dof(self, pose_list, object_title, idx):
        file_path = f'{self.txt_path} {object_title}.txt'
        with open(file_path, 'a') as f:
            for object_idx, pose in enumerate(pose_list[object_title]):
                dof6 = pose['pose']
                ob_name = pose_list[object_title][object_idx]['id']
                # f.write(f'{idx}_{ob_name}_{object_idx} {dof6}\n')
                f.write(f'{idx}_{ob_name}_{object_idx}    {dof6[0]} {dof6[1]} {dof6[2]}    {dof6[3]} {dof6[4]} {dof6[5]} {dof6[6]}\n')


    
    def get_hand_bones_pose(self, bones_list, ax, color):
        for pose in bones_list:
            ax.scatter(pose['pose'][0], pose['pose'][1], pose['pose'][2], s=10, c = color)

    def make_folder(self, folder_dir):
        try:
            if os.path.exists(folder_dir):
                print(f"folder {folder_dir} is exist")
            else:
                os.makedirs(folder_dir)    
        except OSError:
            print('Error: Creating directory. ' + folder_dir)
    
    def set_txtfile(self, txt_object):
        for object_title in txt_object:
            file_path = f'{self.txt_path} {object_title}.txt'
            with open(file_path, 'w') as f:
                f.writelines('')


        
    def load_sequence_path(self, episode_data, file_name):
        
        folder_dir= './result_avi'
        if __name__ == '__main__':
            self.make_folder(folder_dir)
        fig = plt.figure(figsize=(15,5))
        # out = cv2.VideoWriter(f'{folder_dir}/{file_name}.avi',cv2.VideoWriter_fourcc(*'DIVX'), 100, (640,480))
        out = cv2.VideoWriter(f'{folder_dir}/{file_name}.avi',cv2.VideoWriter_fourcc(*'DIVX'), 100, (1500,500))
        # out = cv2.VideoWriter(f'{folder_dir}/{file_name}.avi',cv2.VideoWriter_fourcc(*'DIVX'), 100)
        # for idx, timestamp_data in tqdm(enumerate(episode_data)):

        # get txt path
        self.txt_path = f'{folder_dir}/{file_name}_'
        
        txt_object = ['object_dummy', 'MountingBar', 'hand_L', 'hand_R']
        
        self.set_txtfile(txt_object)

        for idx, timestamp_data in tqdm(enumerate(episode_data), desc=f'{folder_dir}/{file_name}.avi', total=len(episode_data)):
            # get pose, 6dof data txt
            for object_title in txt_object:
                self.get_list_6dof(timestamp_data, object_title, idx)
                # self.get_list_6dof(timestamp_data, object_title, idx)
            


            # make 3d plot 1
            ax = fig.add_subplot(131, projection='3d')      
            # pose --> loc_x, loc_y, loc_z, quat_x, quat_y, quat_z, quat_w 
            plot_title        = timestamp_data['timestamp']
            self.get_list_pose(timestamp_data, 'object_dummy', ax)  # black
            self.get_list_pose(timestamp_data, 'hand_L',       ax)  # chartreuse   
            self.get_list_pose(timestamp_data, 'hand_R',       ax)  # aqua
            self.get_list_pose(timestamp_data, 'MountingBar',  ax)  # navy
            # ax.set_ylim([-150, -30])
            # ax.set_xlim([300, 800])
            # ax.set_zlim([0, 140])
            # ax.set_title(plot_title, fontsize=16)

            # make 3d plot 2
            ax2 = fig.add_subplot(132, projection='3d')
            hand_bones = timestamp_data['hand_L_bones'][0]['bones']
            self.get_hand_bones_pose(hand_bones, ax2, self.color['hand_L'])

            ax2.set_title(plot_title, fontsize=16)
            # ax.set_ylim([-150, -30])
            # ax.set_xlim([300, 800])
            # ax.set_zlim([0, 140])


            # make 3d plot 3
            ax3 = fig.add_subplot(133, projection='3d')
            hand_bones = timestamp_data['hand_R_bones'][0]['bones']
            self.get_hand_bones_pose(hand_bones, ax3, self.color['hand_R'])


            # ax.set_title(plot_title, fontsize=16)
            # ax.set_ylim([-150, -30])
            # ax.set_xlim([300, 800])
            # ax.set_zlim([0, 140])








            img = self.figure_to_array(fig)
            plt.clf()
            out.write(img)

            ## save img file 
            # self.make_folder('save_result')
            # cv2.imwrite(f'save_result/cv2_test{idx}.png', img)
        out.release()


if __name__ == '__main__':
    action_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/ILIAS_SC1%262_3Items.meta.bson'
    bson_path   = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/SC1_HD_5.bson'
    
    a = load_bson()
    tmp = a.get_timestep_data(bson_path, action_path)

    print('finish')
