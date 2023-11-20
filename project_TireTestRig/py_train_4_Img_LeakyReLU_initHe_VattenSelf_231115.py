import sys
sys.path.append('/home/swang597/Documents/Research/Project_heightmap/Utility_projHM')

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

# from sklearn.model_selection import train_test_split
# from sklearn.datasets import fetch_california_housing
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import random

import datetime
from MyDataset import MyDataset, MyDataset_4wheel, read_multiple_dataset
import model_3e3d as mymodel
from NNTrainer import NNTrainer
import argparse
import glob

parser = argparse.ArgumentParser(description='Train 2NNChrono model')
parser.add_argument('--lr', type=float, default=0.01, help='an float for learning rate. Default is 1e-2')
parser.add_argument('--NN_idx', type=int, default=0, help='an NN_idx integer: 0-Img, 1-Vec, 2-(Img,Vec). Default is 0.')
parser.add_argument('--batch_size', type=int, default=64, help='an integer for random seed. Default is 64')
# parser.add_argument('--wheel_cur', type=int, default=10, help='an integer specifying the wheel idx. Default is 10')
parser.add_argument('--seed', type=int, default=1, help='an integer for random seed. Default is 0')
parser.add_argument('--flag_retrain', type=int, default=0, help='Integer flag to determine whether to retrain the model. \
                    Set to 1 for retraining, and 0 for not retraining. Default is 0.')
parser.add_argument('--num_retrain', type=int, default=1, help='Integer specifying the maximum number of models to retrain.\
                     A value of -1 means all available models will be retrained. The default is 1. \
                    Models are selected for retraining based on their loss values, starting with the one with the smallest loss.')
parser.add_argument('--flag_usePredI', type=int, default=1, help='Integer specifying the maximum number of models to retrain.\
                     A value of -1 means all available models will be retrained. The default is 1. \
                    Models are selected for retraining based on their loss values, starting with the one with the smallest loss.')

args = parser.parse_args()

NN_idx = args.NN_idx
learn_rate_init = args.lr
batch_size = args.batch_size
rand_seed = args.seed
flag_retrain = args.flag_retrain
num_retrain = args.num_retrain
# wheel_cur = args.wheel_cur
flag_usePredI = args.flag_usePredI
print(f'Runing .........\n Arguments: NN_idx={NN_idx}, seed={rand_seed}')

torch.manual_seed(rand_seed)
np.random.seed(rand_seed)
random.seed(rand_seed)
# set parameters and keywords
nDT = 1


img_rowsize, img_colsize = 84, 60
# input_vec_colsize = 3
Ksize = 5
padding = 2
poolSize1 = 2
poolSize2 = 3

epoch_start = 0
nepoch = 1000   # number of epochs to run
print(f"Set parameters:nepoch={nepoch},batch_size={batch_size},learn_rate_init={learn_rate_init}")

#Restart training using the best model
# best_mse, best_epoch = 0.00055132, 859 # np.inf, 0   # init to infinity
best_pth_filename = None

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"device is {device}")
# read dataset ----------
fn_dataset_train = f"/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build/DEMO_OUTPUT/Dataset_4_ML_train/"
fn_dataset_test = f"/home/swang597/Documents/Research/chrono_fork_radu/project_TireTestRig/build/DEMO_OUTPUT/Dataset_4_ML_train/"

state_dict_modelImg = "Model/Model_2stepNN_iDT1_img84by60_231113/img_seed12073/state_dict_epoch973_best_mse0.00075724.pth"
# dataset_NN1_train = MyDataset_4wheel(read_multiple_dataset(fn_dataset, filename))
# if wheel_cur == 10:
#     wheel_list = [0,1]
# else:
#     wheel_list = [2,3]

dataset_NN1_train = MyDataset(fn_dataset_train + 'Iin_NN1_ts_train.pt', fn_dataset_train + 'Vec_NN2_ts_train.pt', fn_dataset_train + 'F_NN1_ts_train.pt')
dataset_NN1_test = MyDataset(fn_dataset_test + 'Iin_NN1_ts_test.pt', fn_dataset_test + 'Vec_NN2_ts_test.pt', fn_dataset_test + 'F_NN1_ts_test.pt')

dataset_NN2_train = MyDataset(fn_dataset_train + 'Iin_NN2_ts_train.pt', fn_dataset_train + 'Vec_NN2_ts_train.pt', fn_dataset_train + 'Iout_NN2_ts_train.pt')
dataset_NN2_test = MyDataset(fn_dataset_test + 'Iin_NN2_ts_test.pt', fn_dataset_test + 'Vec_NN2_ts_test.pt', fn_dataset_test + 'Iout_NN2_ts_test.pt')

img_channel_NN1, img_rowsize, img_colsize = dataset_NN1_train[0][0].size()
vec_nfeature_in = dataset_NN1_train[0][1].size()[-1]
vec_nfeature_out = dataset_NN1_train[0][2].size()[-1]
img_channel_NN2 = dataset_NN2_train[0][0].size()[0]    
print("img_channel_NN1, img_rowsize, img_colsize, img_channel_NN2,vec_nfeature_in,vec_nfeature_out")
print(img_channel_NN1, img_rowsize, img_colsize, img_channel_NN2,vec_nfeature_in,vec_nfeature_out )
print(f"*** size of train={len(dataset_NN1_train)}, test={len(dataset_NN1_test)} ***")
torch.manual_seed(rand_seed)

if NN_idx == 1:
    if flag_usePredI:
        model_Img = mymodel.UNet_CNN3e3d_IV_I(img_channel_NN2, img_rowsize, img_colsize, vec_nfeature_in, Ksize, padding, poolSize1, poolSize2)
        model_Img.load_state_dict(torch.load(state_dict_modelImg))
        print(f"*** ***Using predicted dImg model:{state_dict_modelImg}")
        device_model_Img = next(model_Img.parameters()).device
        if device_model_Img.type != device:
            model_Img = model_Img.to(device)

    # model = mymodel.UNet_CNN3e3d_IV_V(img_channel_NN1, img_rowsize, img_colsize, vec_nfeature_in, Ksize, padding, poolSize1, poolSize2, vec_nfeature_out)
    model = mymodel.UNet_LeakyReLU_CNN3e3d_lin6_VattenSelf_V(img_channel_NN1, img_rowsize, img_colsize, vec_nfeature_in, Ksize, padding, poolSize1, poolSize2, vec_nfeature_out)
    NN_keyword = 'vec'
    train_loader = DataLoader(dataset=dataset_NN1_train, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(dataset=dataset_NN1_test, batch_size=batch_size, shuffle=True)
else:
    model = mymodel.UNet_CNN3e3d_IV_I(img_channel_NN2, img_rowsize, img_colsize, vec_nfeature_in, Ksize, padding, poolSize1, poolSize2)
    NN_keyword = 'img'
    train_loader = DataLoader(dataset=dataset_NN2_train, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(dataset=dataset_NN2_test, batch_size=batch_size, shuffle=True)

# Apply the weight initialization
model.apply(mymodel.init_weights)

device_model = next(model.parameters()).device
if device_model.type != device:
    model = model.to(device)

num_parameters = sum(p.numel() for p in model.parameters())
print(f"Load model, Done. Model NN_idx:{NN_idx}, Number of parameters: {num_parameters}")

# ======Define foldername======
today = datetime.datetime.today()
# Format the date as yymmdd
today_str = today.strftime("%y%m%d")

folder_model_wheel = f"Model/Model_2stepNN_iDT{nDT}_img{img_rowsize}by{img_colsize}_{today_str}_flag_usePredI{flag_usePredI}_LeakyReLU_VattenSelf/"
if not os.path.exists(folder_model_wheel):
    os.makedirs(folder_model_wheel)
    print(f"mkdir {folder_model_wheel}")

folder_model= folder_model_wheel + NN_keyword + "_seed"+str(rand_seed)+"/"
if not os.path.exists(folder_model):
    os.makedirs(folder_model)
    print(f"***model folder: {folder_model}")

criterion = nn.MSELoss(reduction='mean') 
optimizer = optim.Adam(model.parameters(), lr=learn_rate_init)

trainer = NNTrainer(model, train_loader, test_loader, criterion, optimizer)
list_loss_train, best_epoch, best_loss_test, need_reduceLr, epoch_reduced, needReduceLr = \
    [], 0, float('inf'), 0, epoch_start, 0
best_model_state_dict = None
for iepoch in range(epoch_start, epoch_start+nepoch):
    if needReduceLr == 1:
        learn_rate_init *= 0.1
        optimizer.lr = learn_rate_init
        needReduceLr, epoch_reduced = 0, iepoch
        print(f"Changed lr: epoch:{iepoch}, lr={learn_rate_init}======================")
    if NN_idx == 1 and flag_usePredI:
        [loss_train] = trainer.train_F(num_epochs=1, model_Img=model_Img)
        [loss_test, _ ] = trainer.evaluate_F(model_Img=model_Img)
    else:
        [loss_train] = trainer.train(num_epochs=1)
        [loss_test, _ ] = trainer.evaluate()
    
    if loss_test < best_loss_test:
        best_epoch, best_loss_test = iepoch, loss_test
        best_model_state_dict = model.state_dict()
        print(f"Find better epoch:{iepoch}, loss_train={loss_train:.8f}, loss_test={loss_test:.8f}")
    if iepoch % 10 == 0:
        print(f"epoch:{iepoch}, loss_train={loss_train:.8f}, loss_test={loss_test:.8f}")
    if (iepoch > 0 and iepoch % (nepoch//10) == 0) or iepoch == (nepoch - 1):
        # trainer.save_state_dict("model_"+str(iepoch)+".pth")
        trainer.save_state_dict(folder_model + f"epoch{iepoch}_state_dict.pth")
        print(f"saved eposh={iepoch}")
    if iepoch - best_epoch > 100 and iepoch - epoch_reduced > 100:
        needReduceLr = 1
    # stop training if no improvement after 200 epoch
    if iepoch - best_epoch > 200:# and iepoch - epoch_start > 300:
        print(f"Stop training since no improvement after 200 epoch, at epoch {iepoch}")
        break
# After the training loop, save the full model with the best state dictionary
if best_model_state_dict is not None:
    print("Before save")
    model.load_state_dict(best_model_state_dict)
    trainer.save_model(folder_model + f"model_epoch{best_epoch}_best_mse{best_loss_test:.8f}.pth")
    trainer.save_state_dict(folder_model + f"state_dict_epoch{best_epoch}_best_mse{best_loss_test:.8f}.pth")
    [loss_test, _ ] = trainer.evaluate()
    print(f"Saved best_epoch:{best_epoch}, loss_train={loss_train:.8f}, loss_test={loss_test:.8f}")
