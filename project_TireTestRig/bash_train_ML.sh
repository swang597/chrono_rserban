#!/bin/bash

# Sleep for 10 seconds
# echo "Sleeping for 7200 seconds... then start"
# sleep 7200
# python py_prepare_data.py

# Start the job
echo "Starting the job."
# Your job command goes here, for example:
# ./your_program

batch_size=128
flag_usePredI=1
# 2wheel-in-one NNs
for i in {1..50}; do
    seed=$((RANDOM))
    # echo python py_train_4_Img_231113.py --flag_usePredI ${flag_usePredI} --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    # python py_train_4_Img_231113.py --flag_usePredI ${flag_usePredI}  --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    #
    # echo python py_train_4_Img_LeakyReLU_231115.py --flag_usePredI ${flag_usePredI} --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    # python py_train_4_Img_LeakyReLU_231115.py --flag_usePredI ${flag_usePredI}  --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    #
    #
    # echo python py_train_4_Img_LeakyReLU_initHe_VattenSelf_231115.py --flag_usePredI ${flag_usePredI} --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    # python py_train_4_Img_LeakyReLU_initHe_VattenSelf_231115.py --flag_usePredI ${flag_usePredI}  --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    #
    # echo python py_train_4_Img_231117.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    # python py_train_4_Img_231117.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    echo python py_train_4_Img_varKsize_231117.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
    python py_train_4_Img_varKsize_231117.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --seed ${seed} # --wheel_cur ${iwheel}
done


# NN_idx=1
# NN1
# for iwheel in 0 1 2 3; do
#     for i in {1..10}; do
#         seed=$((RANDOM))
#         echo python py_train_2NNChrono_230928.py --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#         python py_train_2NNChrono_230928.py --NN_idx 1 --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#         echo python py_train_2NNChrono_230928.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#         python py_train_2NNChrono_230928.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#     done
# done

# NN_idx=1
# # NN1
# for iwheel in 0 3; do
#     for i in {1..10}; do
#         seed=$((RANDOM))
#         echo python py_train_2NNChrono_230925.py --NN_idx ${NN_idx} --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#         python py_train_2NNChrono_230925.py --NN_idx ${NN_idx} --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#     done
# done

# # NN2
# for iwheel in 2; do
#     for i in {1..10}; do
#         seed=$((RANDOM))
#         echo python py_train_2NNChrono_230925.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#         python py_train_2NNChrono_230925.py --NN_idx 2 --lr 0.01 --batch_size ${batch_size} --wheel_cur ${iwheel} --seed ${seed} 
#     done
# done

# NN1
# for iwheel in 2; do
#     for seed in 21036 16847 17879; do
#         # seed=$((RANDOM))
#         echo python py_train_2NNChrono_230918.py --NN_idx 1 --lr 0.01 --batch_size 64 --wheel_cur ${iwheel} --seed ${seed} 
#         python py_train_2NNChrono_230918.py --NN_idx 1 --lr 0.01 --batch_size 64 --wheel_cur ${iwheel} --seed ${seed} 
#     done
# done
