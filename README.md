# This document primarily explains how to operate the code to simulate EKF SLAM and details the functions of each code and the contents of the datasets.

## Dataset
- `SLAM_Error_Data.mat` is the dataset of the robot's actual path and the EKF predicted path with different packet loss rates in intermittent observations.
- `sim_result_original_3.mat` is the dataset of the robot's actual path and the EKF predicted path without packet loss using map 3.
- `sim_result_original.mat` is the dataset of the robot's actual path and the EKF predicted path without packet loss using map 2.
- `sim_result_int.mat` is the currently saved dataset of the robot's actual path and the EKF predicted path using the Loss-Compensated method.
- `sim_result.mat` is the currently saved dataset of the robot's actual path and the EKF predicted path.
- `Int_SLAM_Error_Data.mat` is the dataset of the robot's actual path and the Loss-Compensated EKF predicted path with different packet loss rates in intermittent observations.

## Code
- `slam_error_analysis_compare.m` compares the median errors of using or not using Loss-Compensated EKF under all packet loss conditions.
- `slam_error_analysis.m` compares the EKF SLAM errors under different packet loss probabilities on the same map.
- `simulation_config.m` contains the parameters for the simulation files.
- `Loss_analysis_int.m` calculates and saves the error between the current Loss-Compensated EKF trajectory and the actual trajectory.
- `Loss_analysis.m` calculates and saves the error between the current EKF trajectory and the actual trajectory.
- `Intermittent_analysis.m` compares the errors of Loss-Compensated and normal EKF under the same packet loss rate.
- `EKF_SLAM_simulation.m` simultaneously simulates the trajectories of the actual path, Loss-Compensated, and normal EKF under the same packet loss rate (individual comparison).
- `EKF_lossx_simulation.m` simultaneously simulates the trajectories of the actual path, Loss-Compensated, and normal EKF under the same packet loss rate (three trajectories simultaneously).
- `draw_tendency.m` visualizes the improvement rate.
- `ATE_compare.m` visualizes the ATE.
- `ATE.m` calculates the ATE of the current trajectory.

## Operation Procedure
1. First, modify the packet loss rate `packet_loss_prob` in `EKF_lossx_simulation.m`, then run it. The trajectories for both Loss-Compensated and normal EKF will be saved.
2. Then, separately modify the names and run `Loss_analysis_int.m` and `Loss_analysis.m`.
3. If you need to compare the errors of a specific EKF under all packet loss rates, run `slam_error_analysis.m`.
4. To compare the errors of Loss-Compensated and normal EKF under the same packet loss rate, run `Intermittent_analysis.m`.


##本文档主要说明如何操作代码以模拟EKF SLAM，以及说明各代码功能和数据集内容

##数据集
SLAM_Error_Data.mat 是EKF在不同间隙观测丢包率的机器人真实路径和EKF预测路径数据集
sim_result_original_3.mat 是使用map 3时，没有丢包情况下的机器人真实路径和EKF预测路径数据集
sim_result_original.mat 是使用map 2时，没有丢包情况下的机器人真实路径和EKF预测路径数据集
sim_result_int.mat 是当前保存的使用Loss-Compensated方法的机器人真实路径和EKF预测路径数据集
sim_result.mat 是当前保存的机器人真实路径和EKF预测路径数据集
Int_SLAM_Error_Data.mat 是Loss-Compensated EKF在不同间隙观测丢包率的机器人真实路径和EKF预测路径数据集

##代码
slam_error_analysis_compare.m 所有丢包情况下是否使用Loss-Compensated EKF的误差中位数对比
slam_error_analysis.m 用来对比同一个地图的不同丢包概率下EKF SLAM的误差
simulation_config.m 模拟文件的各个参数
Loss_analysis_int.m 计算当前Loss-Compensated EKF轨迹和真实轨迹的误差并保存
Loss_analysis.m 计算当前EKF轨迹和真实轨迹的误差并保存
Intermittent_analysis.m 同一丢包率下的Loss-Compensated和normal EKF的误差对比
EKF_SLAM_simulation.m 同时模拟相同丢包率下的真实和Loss-Compensated/normal EKF的轨迹 （单独对比）
EKF_lossx_simulation.m 同时模拟相同丢包率下的真实、Loss-Compensated和normal EKF的轨迹 （三个轨迹同时）
draw_tendency.m improvement rate的可视化
ATE_compare.m ATE的可视化
ATE.m 计算当前轨迹的ATE



##操作流程
1.先在EKF_lossx_simulation.m中修改丢包率packet_loss_prob，然后运行，使用Loss-Compensated和normal EKF的轨迹都会保存
2.然后分别修改命名和运行Loss_analysis_int.m和Loss_analysis.m
3.如果需要查看所有丢包率下的特定EKF的误差对比，运行slam_error_analysis.m
4.如果要看同一Loss-Compensated和normal EKF的误差对比，运行Intermittent_analysis.m
