import sys
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
import os
import numpy as np
import tqdm
import torchvision
from torchvision import datasets, models, transforms
import copy

from sklearn.model_selection import train_test_split
from sklearn.datasets import fetch_california_housing
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import random

from datetime import date

def main():
    torch.manual_seed(1)
    np.random.seed(1)
    random.seed(1)
    # Read and normalize data
    foldername_SCM_path = '/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build/DEMO_OUTPUT/'
    terrain_initX_all = [x * 2 for x in range(-46,47)]
    terrain_initH_all = [-1, -1.5, -2] #[x * 0.1 for x in range(-100, 0)]
    normLoad_all = [500, 1000, 2000]

    idx_initX = list(range(len(terrain_initX_all)))
    random.shuffle(idx_initX)

    
    foldername_SCM_dataset_train = foldername_SCM_path + 'Dataset_4_ML_train_diffLoad_231119/'
    foldername_SCM_dataset_test = foldername_SCM_path + 'Dataset_4_ML_test_diffLoad_231119/'
    flag_train = 0
    flag_1traj = 0

    if flag_train:
        # save terrain_initX_train to txt
        if not os.path.exists(foldername_SCM_dataset_train):
            os.makedirs(foldername_SCM_dataset_train)
            print("mkdir "+foldername_SCM_dataset_train)
        terrain_initX_train = [terrain_initX_all[i] for i in idx_initX[:int(len(idx_initX)*0.8)]]
        terrain_initX_test = [terrain_initX_all[i] for i in idx_initX[int(len(idx_initX)*0.8):]]
        print("train:", terrain_initX_train, "test:", terrain_initX_test)

        np.savetxt(foldername_SCM_dataset_train +'terrain_initX_train.txt', terrain_initX_train)
        np.savetxt(foldername_SCM_dataset_train +'terrain_initX_test.txt', terrain_initX_test)
        
        terrain_initX_list = terrain_initX_train
        foldername_SCM_dataset = foldername_SCM_dataset_train
        
    else:
        foldername_SCM_dataset = foldername_SCM_dataset_test
        terrain_initX_list = np.loadtxt(foldername_SCM_dataset_train +'terrain_initX_test.txt')
        if flag_1traj:
            terrain_initX_list = [terrain_initX_test[0]]
            terrain_initH_all = [terrain_initH_all[-1]]
            foldername_SCM_dataset = foldername_SCM_dataset_test[:-1] + f"_1traj_initX{terrain_initX_list[0]:.1f}_initH{terrain_initH_all[0]:.1f}/"
    print("for loop param:",terrain_initH_all, terrain_initX_list)
    
    if not os.path.exists(foldername_SCM_dataset):
        os.makedirs(foldername_SCM_dataset)
        print("mkdir "+foldername_SCM_dataset)

    flag_save_pt = True
    if flag_save_pt:
        # Save tensor data to .pt files
        foldername_dataPT = foldername_SCM_dataset
        
    else:
        # Save tensor data to .txt files
        foldername_dataPT = foldername_SCM_dataset[:-1] + "_txt/"

    print(f"folder for dump is {foldername_dataPT}")
    if not os.path.exists(foldername_dataPT):
        os.makedirs(foldername_dataPT)
        print("mkdir "+foldername_dataPT)

    idx_XVW = [3,8,10,12] # [z, vx, vz, wy]. {time, x,y,z, RR,Rx,Ry,Rz, vx,vy,vz, wx,wy,wz}
    idx_FT = [4,6,8] # [Fx,Fz,Ty]. {time, x,y,z, Fx,Fy,Fz, Tx,Ty,Tz}

    dt=5e-4
    terrain_grid=0.005
    # time_tot=1.5
    dt_HM=dt*10 # 
    I00, I10, Vec1, dF1, F1, data_time0 = [], [], [], [], [], []

    # Set datum as terrain_initH = -1
    for normLoad in normLoad_all:
        print(f"normLoad={normLoad}")
        for terrain_initH in terrain_initH_all: # [x * 0.5 for x in range(-10, -1)]:  # -5 to -0.7 with step 0.5
            print(f"terrain_initH={terrain_initH}")
            for terrain_initX in terrain_initX_list: # [x * 0.5 for x in range(-46, 47)]:  # -23 to 23 with step 0.5. Use [15-23] for test
                dH_terrain = -1 - terrain_initH # offset of all z direction displacement
                # print(f"terrain_initX={terrain_initX}, terrain_initH={terrain_initH}")
                if normLoad == 1000:
                    foldername_SCM_name = f"TIRE_TEST_RIG_dt{dt:.6f}_terrGrid{terrain_grid:.6f}terrX{terrain_initX:.6f}terrH{terrain_initH:.6f}/"
                else:
                    foldername_SCM_name = f"TIRE_TEST_RIG_dt{dt:.6f}_terrGrid{terrain_grid:.6f}terrX{terrain_initX:.6f}terrH{terrain_initH:.6f}normLoad{normLoad:.6f}/"
                
                
                    
                foldername_SCM = foldername_SCM_path + foldername_SCM_name
                if not os.path.exists(foldername_SCM):
                    # print(f"foldername_SCM={foldername_SCM} does not exist. Continue")
                    continue
                # print(f"foldername_SCM={foldername_SCM} found.")

                nDT = 1
                filename_XVW = foldername_SCM + 'ROVER_states_saved.txt' # time,pos,rot,lin_vel,ang_vel,omega
                filename_FT = foldername_SCM + 'SCM_force_saved.txt' # time,point,force,torque

                # nFrame_lim = 1002*4
                # frame_idx = [i for i in range(0*4, 1005*4)]
                data_XVW_all = np.loadtxt(filename_XVW)#[frame_idx,:]
                data_XVW_all[:,3] += dH_terrain # offset of all z direction displacement
                # print("data_XVW_all[0,3]:", data_XVW_all[0,3])
                data_FT_all = np.loadtxt(filename_FT)#[frame_idx,:]
                data_time = data_XVW_all[:,0]
                DT = data_time[1] - data_time[0]
                # print(f"DT={DT:.6f}")

                img_rowsize = 84
                img_colsize = 60

                data_XVW = data_XVW_all[:,idx_XVW]
                data_FT = data_FT_all[:,idx_FT]

                nFrame = int(np.shape(data_time)[0])
                iDT = 1
                for i in range(0,nFrame): #range(200): #
                # Filename: # hmap_wheel3_0.19_input_iDT1.npy hmap_wheel3_0.19_output_iDT1.npy
                    if i+iDT >= nFrame:
                        continue

                    # if the wheel is not in contact with the ground, skip. 
                    # Radius of wheel is 0.208, Hmax of terrain is 0.1
                    # if data_XVW[i + iDT,0] - terrain_initH > 0.208 + 0.1 + 0.02:
                    if data_XVW[i + iDT,0] > 0.208 + 0.1 + 0.02:
                        continue

                    if terrain_initH > -1.5 + 1e-3 or terrain_initH < -1.5 - 1e-3:
                        if data_XVW[i + iDT,1] > 0.5:
                            continue

                    data_time0.append(data_time[i])
                    
                    Vec1.append(data_XVW[i + iDT ,:])
                    # dF1 = F1 - F0
                    F1.append(data_FT[i+iDT,:])
                    dF1.append(data_FT[i+iDT,:] - data_FT[i,:])
                    
                    #I00
                    filename_I00 = foldername_SCM + f"hmap_Pat{data_time[i]:.6f}_Tat{data_time[i]:.6f}.txt"
                    I00_i = np.loadtxt(filename_I00) + dH_terrain
                    nrow, ncol = np.shape(I00_i)
                    I00_i = I00_i[(nrow-img_rowsize)//2:(nrow+img_rowsize)//2,(ncol-img_colsize)//2:(ncol+img_colsize)//2]
                    I00.append(I00_i)
                    #I10
                    filename_I10 = foldername_SCM + f"hmap_Pat{data_time[i]:.6f}_Tat{data_time[i+iDT]:.6f}.txt"
                    I10_i = np.loadtxt(filename_I10) + dH_terrain
                    nrow, ncol = np.shape(I10_i)
                    I10_i = I10_i[(nrow-img_rowsize)//2:(nrow+img_rowsize)//2,(ncol-img_colsize)//2:(ncol+img_colsize)//2]
                    I10.append(I10_i)

                    # print("I00_i[0,0], I10_i[0,0], I10_i[0,0]-I00_i[0,0]",I00_i[0,0], I10_i[0,0], I10_i[0,0]-I00_i[0,0])
                print(f"i={i},terrain_initX={terrain_initX}, terrain_initH={terrain_initH}, len(I00)={len(I00)}, vx={data_XVW[i,1]},vx={Vec1[-1][1]}, vz={Vec1[-1][2]}, wy={Vec1[-1][3]}")

    # constract other data
    I00, I10 = np.array(I00), np.array(I10)
    dI10 = I10 - I00
    Vec1 = np.array(Vec1)
    dF1 = np.array(dF1)
    F1 = np.array(F1)
    data_time0 = np.array(data_time0)
    print(np.shape(I00), np.shape(I10), np.shape(dI10), np.shape(Vec1), np.shape(dF1), np.shape(F1), np.shape(data_time0))
    # scaler_dX = MinMaxScaler()
    # scaler_dF = MinMaxScaler()
    # dX2_nm = scaler_dX.fit_transform(dX2)
    # dF1_nm =scaler_dF.fit_transform(dF1)
    if flag_train:
        min_Vec, max_Vec = np.min(Vec1,axis=0), np.max(Vec1,axis=0)
        min_dF, max_dF = np.min(dF1,axis=0), np.max(dF1,axis=0)
        min_F, max_F =   np.min(F1,axis=0),  np.max(F1,axis=0)
        min_I, max_I = np.min(I00), np.max(I00)
        min_dI, max_dI = np.min(dI10), np.max(dI10)

        np.savetxt(foldername_dataPT +'Vec_min_max.txt', [min_Vec, max_Vec])
        np.savetxt(foldername_dataPT +'dF_min_max.txt', [min_dF, max_dF])
        np.savetxt(foldername_dataPT +'F_min_max.txt', [min_F, max_F])
        np.savetxt(foldername_dataPT +'I_min_max.txt', [min_I, max_I])
        np.savetxt(foldername_dataPT +'dI_min_max.txt', [min_dI, max_dI])

    else:
        [min_Vec, max_Vec] = np.loadtxt(foldername_SCM_dataset_train +'Vec_min_max.txt')
        [min_dF, max_dF]= np.loadtxt(foldername_SCM_dataset_train +'dF_min_max.txt')
        [min_F, max_F] = np.loadtxt(foldername_SCM_dataset_train +'F_min_max.txt')
        [min_I, max_I] = np.loadtxt(foldername_SCM_dataset_train +'I_min_max.txt')
        [min_dI, max_dI] = np.loadtxt(foldername_SCM_dataset_train +'dI_min_max.txt')
        print("min_Vec, max_Vec", min_Vec, max_Vec)
        print("min_dF, max_dF", min_dF, max_dF)
        print("min_F, max_F", min_F, max_F)
        print("min_I, max_I", min_I, max_I)
        print("min_dI, max_dI", min_dI, max_dI)

        
    Vec1_nm = (Vec1 - min_Vec) / (max_Vec - min_Vec)
    dF1_nm = (dF1 - min_dF) / (max_dF - min_dF)
    F1_nm = (F1 - min_F) / (max_F - min_F)

    I00_nm = (I00 - min_I) / (max_I - min_I) 
    I10_nm = (I10 - min_I) / (max_I - min_I) 
    dI10_nm = (dI10 - min_dI) / (max_dI - min_dI)
                
    # Define a lambda function to perform type conversion and unsqueezing
    convert_and_unsqueeze = lambda x: torch.from_numpy(x).type(torch.float32).unsqueeze(1)
    convert_np2ts = lambda x: torch.from_numpy(x).type(torch.float32)
    # Convert and modify NumPy arrays to PyTorch tensors in one line
    I00_ts, I10_ts, dI10_ts = \
        map(convert_and_unsqueeze, (I00_nm, I10_nm, dI10_nm))
    Vec1_ts, dF1_ts, F1_ts = map(convert_np2ts, (Vec1_nm, dF1_nm, F1_nm))

    I_NN1_ts = torch.cat([I00_ts, dI10_ts], dim = 1)

    # print(I_NN1_ts.size(), dF1_ts[:-1].size(), np.shape(data_time0[:-1]))
    if flag_1traj:
        Iin_NN1_ts_test, dF_NN1_ts_test, F_NN1_ts_test, Iin_NN2_ts_test, Vec_NN2_ts_test, Iout_NN2_ts_test, I10_NN2_ts_test, time_test = \
        I_NN1_ts,        dF1_ts,         F1_ts,         I00_ts,          Vec1_ts,         dI10_ts,          I10_ts,          data_time0
    else:
        rand_seed = 1
        Iin_NN1_ts_train, Iin_NN1_ts_test, dF_NN1_ts_train, dF_NN1_ts_test, F_NN1_ts_train, F_NN1_ts_test, \
        Iin_NN2_ts_train, Iin_NN2_ts_test, Vec_NN2_ts_train, Vec_NN2_ts_test, Iout_NN2_ts_train, Iout_NN2_ts_test, \
        I10_NN2_ts_train, I10_NN2_ts_test, \
            time_train, time_test = \
            train_test_split(I_NN1_ts, dF1_ts, F1_ts, \
                            I00_ts, Vec1_ts, dI10_ts, \
                            I10_ts,\
                                data_time0, test_size=0.2, random_state=rand_seed)

        print(Iin_NN1_ts_train.size(), dF_NN1_ts_train.size(), F_NN1_ts_train.size(), Iin_NN2_ts_train.size(), Vec_NN2_ts_train.size(), Iout_NN2_ts_train.size(), np.shape(time_train))
    print(Iin_NN2_ts_test.size(), Vec_NN2_ts_test.size(), np.shape(time_test))

    


    if flag_save_pt:
        # Saving as PyTorch tensors
        if not flag_1traj:
            torch.save(Iin_NN1_ts_train, foldername_dataPT +'Iin_NN1_ts_train.pt')
            torch.save(dF_NN1_ts_train, foldername_dataPT +'dF_NN1_ts_train.pt')
            torch.save(F_NN1_ts_train, foldername_dataPT +'F_NN1_ts_train.pt')
            torch.save(Iin_NN2_ts_train, foldername_dataPT +'Iin_NN2_ts_train.pt')
            torch.save(Vec_NN2_ts_train, foldername_dataPT +'Vec_NN2_ts_train.pt')
            torch.save(Iout_NN2_ts_train, foldername_dataPT +'Iout_NN2_ts_train.pt')
            torch.save(I10_NN2_ts_train, foldername_dataPT +'I10_NN2_ts_train.pt')
        

        torch.save(Iin_NN1_ts_test, foldername_dataPT  +'Iin_NN1_ts_test.pt')
        torch.save(dF_NN1_ts_test, foldername_dataPT +'dF_NN1_ts_test.pt')
        torch.save(F_NN1_ts_test, foldername_dataPT +'F_NN1_ts_test.pt')
        torch.save(Iin_NN2_ts_test, foldername_dataPT  +'Iin_NN2_ts_test.pt')
        torch.save(Vec_NN2_ts_test, foldername_dataPT +'Vec_NN2_ts_test.pt')
        torch.save(Iout_NN2_ts_test, foldername_dataPT +'Iout_NN2_ts_test.pt')
        torch.save(I10_NN2_ts_test, foldername_dataPT  +'I10_NN2_ts_test.pt')
        
        
    else:  
        # Saving as PyTorch tensors
        np.save(foldername_dataPT +'Iin_NN1_ts_train.npy', Iin_NN1_ts_train.numpy())
        np.save(foldername_dataPT +'dF_NN1_ts_train.npy', dF_NN1_ts_train.numpy())
        np.save(foldername_dataPT +'F_NN1_ts_train.npy', F_NN1_ts_train.numpy())
        np.save(foldername_dataPT +'Iin_NN2_ts_train.npy', Iin_NN2_ts_train.numpy())
        np.save(foldername_dataPT +'Vec_NN2_ts_train.npy', Vec_NN2_ts_train.numpy())
        np.save(foldername_dataPT +'Iout_NN2_ts_train.npy', Iout_NN2_ts_train.numpy())
        

        np.save(foldername_dataPT  +'Iin_NN1_ts_test.npy', Iin_NN1_ts_test.numpy())
        np.save(foldername_dataPT +'dF_NN1_ts_test.npy', dF_NN1_ts_test.numpy())
        np.save(foldername_dataPT +'F_NN1_ts_test.npy', F_NN1_ts_test.numpy())
        np.save(foldername_dataPT  +'Iin_NN2_ts_test.npy', Iin_NN2_ts_test.numpy())
        np.save(foldername_dataPT +'Vec_NN2_ts_test.npy', Vec_NN2_ts_test.numpy())
        np.save(foldername_dataPT +'Iout_NN2_ts_test.npy', Iout_NN2_ts_test.numpy())

    if not flag_1traj:
        np.save(foldername_dataPT +'time_train.npy', time_train)
    np.save(foldername_dataPT +'time_test.npy', time_test)

    print(f"Done. Train size={np.shape(time_train)}, Test size={np.shape(time_test)}")
    
if __name__ == "__main__":
    main()
