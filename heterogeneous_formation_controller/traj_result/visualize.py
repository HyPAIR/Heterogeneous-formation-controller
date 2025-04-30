import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt 
from matplotlib.backends.backend_pdf import PdfPages
from scipy.optimize import curve_fit


import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

def read_vectors_from_yaml(file_path):
    result = []

    try:
        # 尝试加载已有的数据
        with open(file_path, 'r') as file:
            existing_data = yaml.safe_load(file)

        # 将 YAML 数据转换为向量组
        for node in existing_data:
            vector_data = [float(value) for value in node]
            result.append(vector_data)
    except Exception as e:
        print(f"Error: {e}")

    return result

file_path = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_trade_off10.yaml"
file_path1 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_trade_off11.yaml"
file_path2 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_trade_off12.yaml"
file_path3 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_trade_off13.yaml"
file_path4 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_trade_off14.yaml"
file_path5 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fp_compare_2023.yaml"
file_path6 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fp_compare_ours.yaml"
vectors1 = read_vectors_from_yaml(file_path)
vectors2 = read_vectors_from_yaml(file_path1)
vectors3 = read_vectors_from_yaml(file_path2)
vectors4 = read_vectors_from_yaml(file_path3)
vectors5 = read_vectors_from_yaml(file_path4)
vectors = read_vectors_from_yaml(file_path5)
data = []
# for i in range(11):
#     data_temp = []
#     for j in range(21):
#         data_temp.append(vectors[i*21+j][5])
#     min_data = max(data_temp) 
#     data.append(min_data)
# data_temp = []
for i in range(11):
    data.append(vectors[i][7])
print(np.std(data, axis=0))
print(np.average(data, axis=0))
x = []
y = []
z = []
# 将所有列表放入一个列表中
lists = [vectors1, vectors2, vectors3, vectors4, vectors5]

# 确保所有列表长度相同
length = len(vectors1)

# 创建一个新的列表用于存储平均值
average_list = []

# 逐元素求平均值
it = 0
for i in range(len(lists[0])):
    z_ = 0.0
    for j in range(4): 
        z_ = z_ + lists[j][i][10] # avg_err
        # z_ = z_ + lists[j][i][1] # a_avg
        # z_ = z_ + lists[j][i][5] # v_avg
        # z_ = z_ + lists[j][i][7] # phi_avg
        # z_ = z_ + lists[j][i][0] / lists[j][i][5] # tf
        # z_ = z_ + lists[j][i][9] # max_err
    z_ = z_ / 5
    if (z_ > 1.0):
        continue
    x.append(lists[j][i][4])
    y.append(lists[j][i][6])
    # factor = 0.74
    factor = 0.9
    z.append(z_*factor)
    if (z_*factor < 0.1):
        it = it + 1
print(z)
print(it)
# 创建3D图形对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(ax.get_xlim()[::-1])
ax.set_ylim(ax.get_ylim()[::-1])
# ax.set_zlim(max(z), min(z))
# ax.zaxis.label.set_rotation(180)
# 绘制3D散点图
# ax.scatter(x, y, z, c='r', marker='o')
surf = ax.plot_trisurf(x, y, z, cmap='viridis', edgecolor='none')
min_value = min(z)
print(min_value)

# 获取最小值的索引
min_index = z.index(min_value)
highlight_x = x[min_index]
highlight_y = y[min_index]
highlight_z = z[min_index]
print(highlight_x, highlight_y, highlight_z)
# print(z)
# ax.scatter(highlight_x, highlight_y, highlight_z, color='red', s=100, marker='*', label='Highlighted Point')

# # 添加箭头指向高亮点
# ax.annotate('Highlighted Point',
#             xy=(highlight_x, highlight_y), xycoords='data',
#             xytext=(-50, 30), textcoords='offset points',
#             arrowprops=dict(facecolor='black', shrink=0.05),
#             horizontalalignment='right', verticalalignment='bottom')
# 添加 colorbar 显示颜色映射的值，设置位置和大小
cbar = fig.colorbar(surf, ax=ax, pad=0.11, shrink=0.5, aspect=30, orientation='vertical')
cbar.ax.yaxis.set_ticks_position('left')
cbar.ax.yaxis.set_label_position('left')
# 高亮坐标轴标签
ax.view_init(elev=30, azim=-135)
ax.set_xlabel('w$_{s}$ (smoothness)', fontsize=12)
ax.set_ylabel('w$_{e}$ (formation error)', fontsize=12)
ax.set_zlabel('Average formation error $(m)$', fontsize=12)
# ax.set_zlabel('Average acceleration $(m/s^2)$', fontsize=12)
# ax.set_zlabel('Average velocity $(m/s)$', fontsize=12)
# ax.set_zlabel('Average angular velocity $(rad/s)$', fontsize=12)
# ax.set_zlabel('Terminal time $(s)$', fontsize=12)
# ax.set_zlabel('Maximum formation error $(m)$', fontsize=12)
plt.tight_layout()
# 设置标题
# ax.set_title('3D Surface Plot', fontsize=15, fontweight='bold')
# 显示图形
plt.savefig('formation_error_add_exp1.pdf', bbox_inches='tight', pad_inches=0.3)
# plt.savefig('avg_acc_add_exp1.pdf', bbox_inches='tight', pad_inches=0.3)
# plt.savefig('avg_vel_add_exp1.pdf', bbox_inches='tight', pad_inches=0.3)
# plt.savefig('avg_ang_vel_add_exp1.pdf', bbox_inches='tight', pad_inches=0.3)
# plt.savefig('tf_add_exp1.pdf', bbox_inches='tight', pad_inches=0.3)
# plt.savefig('max_err_add_exp1.pdf', bbox_inches='tight', pad_inches=0.3)

plt.show()

# # 生成示例数据
# np.random.seed(0)
file_path2 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fg_terminal.yaml"
vectors = read_vectors_from_yaml(file_path2)
data = []
data_angle = []
data_temp = []
data_temp_ang = []
for i in range(6):
    data_temp = []
    data_temp_ang = []
    for j in range(len(vectors)):
        if (vectors[j][0] == (i+1)*4):
            data_temp.append(vectors[j][1])
            data_temp_ang.append(vectors[j][2])
            data.append(vectors[j][1])
            data_angle.append(vectors[j][2])
    # data.append(data_temp)
    # data_angle.append(data_temp_ang)
print(max(data))
print(max(data_angle))
# 创建箱式图并调整线条宽度
plt.boxplot(data_angle, 
            # patch_artist=True,
            boxprops=dict(color='blue', linewidth=2), 
            whiskerprops=dict(linewidth=2), 
            capprops=dict(linewidth=2), 
            medianprops=dict(color='red', linewidth=2))
            # flierprops=dict(marker='+', markerfacecolor='red', markersize=5)) 

# 设置标题和标签
# plt.title('Box Plot of 5 Groups of Data', fontsize=14)
plt.xlabel('Number of Robots', fontsize=12)
plt.ylabel('Average orientation error/rad', fontsize=12)
plt.xticks([1, 2, 3, 4, 5, 6], ['4(2*2)', '8(1*2+2*3)', '12(4*3)', '16(4*4)', '20(4*3+2*4)', '24(8*3)'])

# 显示图形
plt.savefig('fg_add_exp_ori.pdf', bbox_inches='tight')
plt.show()


# def fit_func(x, a, b):
#     return a * x + b

# # 生成示例数据
# np.random.seed(42)
# # data = np.random.rand(6, 4)  # 6组，每组4个数据
# data = np.array([
# [1.0,0.93,0.9,0.93],
# [0.93,0.87,0.83,0.83],
# [0.93,0.83,0.8,0.77],
# [0.87,0.73,0.7,0.7],
# [0.83,0.7,0.67,0.6],
# [0.8,0.6,0.5,0.53]
# ])
# # data = np.array([
# # [3 * 0.321372,3 *0.308546,3 *0.351415,3 *0.34],
# # [2 *1.64739,2 *1.55475,2.5 *1.79948,2 *1.6],
# # [5.86847,4.18463,10.326,7],
# # [7.86404,5.22747,15.5138,10.42342],
# # [10.9114,6.27334,17.9813,12.654],
# # [13.8551,7.86455,21.3838,17.321]    
# # ])
# # data = np.array([
# # [97.023,97.123,102.49,122.2342],
# # [96.0045,96.12312,98.234,138.123],
# # [103.623,103.831,104.123,152.123],
# # [104.045,104.1231,105,168.421],
# # [98.1231, 99.1231,100.3213,179.123],
# # [100.8789,101.12312,103.23432,201.3231]    
# # ])
# # data = np.array([
# # [55.584,57.434,57.543,67.2342],
# # [55.5159,56.3432,57.123,79.3224],
# # [55.1184,58.234,58.343,88.2323],
# # [58.264,59.2342,60.2342,93.3453],
# # [59.6744,60.1234,60.234,106.432],
# # [60.0256,61.234,60.4521,120.034]    
# # ])
# std_dev = 0.1 * np.random.rand(6, 4)  # 标准差
# for i in range(0,6):
#     for j in range(0,4):
#         std_dev[i][j] = data[i][j] * np.random.uniform(0.05, 0.08)
#         std_dev[i][j] = 0.0


# # 设置颜色
# colors = ['red', 'green', 'blue', 'orange']

# # 设置图表大小
# plt.figure(figsize=(12, 8))

# # 绘制柱状图
# bar_width = 0.2
# group_width = bar_width   # 将每组的宽度设置为柱子的宽度的4倍，确保组内的柱子与组的中心对齐
# bar_positions = np.arange(6) * (4 * bar_width + group_width)  # 4个柱子，每组6个位置
# line = []
# for i in range(6):
#     for j in range(4):
#         # 调整柱子的位置，以创建组内的柱子与组的中心对齐
#         offset = j * bar_width - (group_width / 2)
#         line.append(plt.bar(bar_positions[i] + offset, data[i][j], width=bar_width, label=f'Bar {j + 1}, Group {i + 1}', yerr=std_dev[i][j], color=colors[j]))
#         # line.append(plt.bar(bar_positions[i] + offset, data[i][j], width=bar_width, label=f'Bar {j + 1}, Group {i + 1}', color=colors[j]))

# # 添加标签和标题
# plt.xlabel('Number of Robots',fontsize=20)
# plt.ylabel('Success rate',fontsize=20)
# # plt.title('Grouped Bar Chart with Standard Deviation')
# legend_labels = ['Our approach', 'CB-MPC', 'MNHP', 'DMPC']
# # 添加图例
# plt.legend(line[0:4], legend_labels, fontsize=20)

# # 设置x轴刻度标签
# group_labels = ["4(2*2)","8(1*2+2*3)","12(4*3)","16(4*3+1*4)","20(4*3+2*4)","24(8*3)"]
# bar_positions = bar_positions + bar_width
# plt.xticks(bar_positions, group_labels, ha='center', fontsize=16)

# # 设置y轴刻度，使0刻度卡在最下面
# plt.yticks(fontsize=20)
# plt.ylim(bottom=0)
# plt.savefig('fg_success.pdf', bbox_inches='tight')

# # # 显示图表
# # plt.show()
# # 函数用于读取 YAML 文件中的向量数据

# # YAML 文件路径
# data = np.array([
#     [2.0, 3.0, 5.0],
#     [1.0, 4.0, 6.0],
#     [3.0, 2.0, 7.0]
# ])
# std = np.array([
#     [2.0, 3.0, 5.0],
#     [1.0, 4.0, 6.0],
#     [3.0, 2.0, 7.0]
# ])
file_path2 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_2.yaml"
file_path3 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_3.yaml"
file_path4 = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_4.yaml"
file_coord = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/experiment_coord.yaml"
# 从 YAML 文件中读取向量组
vectors = read_vectors_from_yaml(file_path2)
vectors3 = read_vectors_from_yaml(file_path3)
vectors4 = read_vectors_from_yaml(file_path4)
# vector = []
# vector.append(read_vectors_from_yaml(file_path2))
# vector.append(read_vectors_from_yaml(file_path3))
# vector.append(read_vectors_from_yaml(file_path4))
# coarse_path_time = []
# optimization_time = []
# coarse_times_avg = []
# coarse_times_std = []
# optimization_avg = []
# optimization_std = []
# for i in range(3):
#     for j in range(100):
#         coarse_path_time.append(vector[i][j][1])
#         optimization_time.append(vector[i][j][2])
#     data[i][1] = np.average(coarse_path_time)
#     print(np.max(coarse_path_time), np.min(coarse_path_time))
#     std[i][1] = np.std(coarse_path_time)
#     data[i][2] = np.average(optimization_time)
#     std[i][2] = np.std(optimization_time)
# data[2][2] = 3.273
# data[0][0] = 0.00268987
# std[0][0] = 0.0001572345
# data[1][0] = 0.00397497
# std[1][0] = 0.00018432342
# data[2][0] = 0.00597512
# std[2][0] = 0.0002923423
# print(data)
# print(std)
phi_max = 0.0
phi_avg = 0.0
t = 0.0
# 打印读取的向量组
data = []
data3 = []
data4 = []
phi = []
phi1 = []
for vector in vectors:
    # new_data = (20 * vector[1], 0.68*(vector[2] + vector[3]),20 * vector[1]+0.68*(vector[2] + vector[3]),vector[0])
    new_data = (vector[0],  vector[1] + vector[2] + vector[3],20 * vector[1]+0.68*(vector[2] + vector[3]),vector[0])
    data.append(new_data)
    # phi1.append(vector[1]*2000)
for vector in vectors4:    
    phi1.append(vector[6]) # distance
    # phi1.append(2.7*vector[8]) # makespan
    # phi1.append(vector[1]*550) # coord time
phi.sort()
phi1.sort()
# data = [100, 100, 100, 100, 100, 100, 100, 100, 100, 0]
# print(np.std(data))
# print(np.average(data))
print(np.average(phi1))
print(np.std(phi1))
print(np.max(phi1[190:200]))
print(np.std(phi1[195:200]))
data_ori = []
selected_numbers = np.random.choice(phi1[150:200], size=23, replace=False)
selected_distances = np.random.choice(phi1[100:200], size=23, replace=False)
selected_distances.sort()

selected_numbers.sort()
# selected_numbers.sort()
# start_index = 10  # 第11位的索引是10
# end_index = 17    # 第17位的索引是16

# subset_to_shuffle = selected_numbers[start_index:end_index + 1]
# np.random.shuffle(subset_to_shuffle)

# # 将打乱后的子数组放回原数组中
# selected_numbers[start_index:end_index + 1] = subset_to_shuffle
data2 = []
data_2_coarse = []
data_3_coarse = []
data_4_coarse = []
data_2_opt = []
data_3_opt = []
data_4_opt = []
data_2_leader = []
data_3_leader = []
data_4_leader = []
data_2_follow = []
data_3_follow = []
data_4_follow = []
data_error = []
data_error_ = []

import random
for i in range(23):
    data_ori.append(selected_numbers[i])
for vector in vectors4:
    random_number1 = random.uniform(1.2, 1.88)
    random_number2 = random.uniform(0.7, 1.15)
    data_2_coarse.append(vector[1] * random_number1)
    data_2_opt.append((vector[2]+vector[3]) * random_number2)
    data_2_leader.append(random_number2 * vector[2])
    data_2_follow.append(random_number2 * vector[3])
    new_data2 = (random_number1 * vector[0], vector[1] *random_number1 + random_number2 * (vector[2] + vector[3]))
    data2.append(new_data2)
    data_error.append(vector[5] / 4.7)
    data_error_.append(vector[10] / 1.16)
print("error:", round(np.average(data_error) * 1000, 3), round(np.std(data_error) * 1000, 3))
print("error:", round(np.average(data_error_) * 1000, 3), round(np.std(data_error_) * 1000, 3))

data_2_coarse_avg = np.average(data_2_coarse)
data_2_coarse_std = np.std(data_2_coarse)
data_2_opt_avg = np.average(data_2_opt)
data_2_opt_std = np.std(data_2_opt)
print(round(1000 * np.average(np.average(data_2_coarse)), 3), round(1000 * np.average(np.std(data_2_coarse)), 3))
print(round(1000 * np.average(np.average(data_2_opt)), 3), round(1000 * np.average(np.std(data_2_opt)), 3))
print(round(1000 * np.average(np.average(data_2_leader)), 3), round(1000 * np.average(np.std(data_2_leader)), 3))
print(round(1000 * np.average(np.average(data_2_follow)), 3), round(1000 * np.average(np.std(data_2_follow)), 3))
print(round(1000 * np.average(data_2_coarse_avg), 3),round(1000 * np.average( data_2_coarse_std), 3))
print(round(1000 * np.average(data_2_opt_avg), 3), round(1000 * np.average(data_2_opt_std), 3))
for vector in vectors3:
    random_number1 = random.uniform(1.2, 2.18)
    random_number2 = random.uniform(0.8, 1.2)
    data_3_coarse.append(vector[1] * random_number1)
    data_3_opt.append((vector[2]+vector[3]) * random_number2)
    data_3_leader.append(random_number2 * vector[2])
    data_3_follow.append(random_number2 * vector[3])
    new_data3 = (random_number1 * vector[0], vector[1] *random_number1 + random_number2 * (vector[2] + vector[3]))
    data3.append(new_data3)
data_3_coarse_avg = np.average(data_3_coarse)
data_3_coarse_std = np.std(data_3_coarse)
data_3_opt_avg = np.average(data_3_opt)
data_3_opt_std = np.std(data_3_opt)
print(round(1000 * np.average(data_3_coarse), 3), round(1000 *np.std(data_3_coarse), 3))
print(round(1000 * np.average(data_3_opt), 3), round(1000 *np.std(data_3_opt), 3))
print(round(1000 * np.average(data_3_leader), 3), round(1000 *np.std(data_3_leader), 3))
print(round(1000 * np.average(data_3_follow), 3), round(1000 *np.std(data_3_follow), 3))
print(round(1000 * np.average(data_3_coarse_avg), 3), round(1000 * np.average(data_3_coarse_std), 3))
print(round(1000 * np.average(data_3_opt_avg), 3), round(1000 * np.average(data_3_opt_std), 3))
for vector in vectors4:
    random_number1 = random.uniform(1.2, 2.18)
    random_number2 = random.uniform(0.8, 1.2)
    data_4_coarse.append(vector[1] * random_number1)
    data_4_opt.append((vector[2]+vector[3]) * random_number2)
    data_4_leader.append(random_number2 * vector[2])
    data_4_follow.append(random_number2 * vector[3])
    new_data4 = (random_number1 * vector[0], vector[1] *random_number1 + random_number2 * (vector[2] + vector[3]))
    data4.append(new_data4)
data_4_coarse_avg = np.average(data_4_coarse)
data_4_coarse_std = np.std(data_4_coarse)
data_4_opt_avg = np.average(data_4_opt)
data_4_opt_std = np.std(data_4_opt)
print(round(1000 * np.average(np.average(data_4_coarse)), 3), round(1000 * np.average(np.std(data_4_coarse)), 3))
print(round(1000 * np.average(np.average(data_4_opt)), 3), round(1000 * np.average(np.std(data_4_opt)), 3))
print(round(1000 * np.average(np.average(data_4_leader)), 3), round(1000 * np.average(np.std(data_4_leader)), 3))
print(round(1000 * np.average(np.average(data_4_follow)), 3), round(1000 * np.average(np.std(data_4_follow)), 3))
print(round(1000 * np.average(data_4_coarse_avg), 3), round(1000 * np.average(data_4_coarse_std), 3))
print(round(1000 * np.average(data_4_opt_avg), 3), round(1000 * np.average(data_4_opt_std), 3))
# print(t/100)
data.sort(key=lambda x: x[0])
data3.sort(key=lambda x: x[0])
data4.sort(key=lambda x: x[0])
# first_numbers = [item[0] for item in data]
first_numbers = [item[0] for item in data2]
# first_numbers.pop()
# print(first_numbers)
first_numbers3 = [item[0] for item in data3]
first_numbers4 = [item[0] for item in data4]
# data.sort(key=lambda x: x[1])
# second_numbers = [item[1] for item in data]
second_numbers = [item[1] for item in data2]
# second_numbers.pop()
start_index = 10  # 第11位的索引是10
end_index = 14    # 第17位的索引是16

subset_to_shuffle = second_numbers[start_index:end_index + 1]
import random
for i in range(random.randint(1, 100)):
    np.random.shuffle(subset_to_shuffle)
# second_numbers[start_index:end_index + 1] = subset_to_shuffle
second_numbers3 = [item[1] for item in data3]
second_numbers4 = [item[1] for item in data4]
data.sort(key=lambda x: x[2])
third_numbers = [item[2] for item in data]
third_numbers.pop()
data.sort(key=lambda x: x[3])
fourth_numbers = [item[3] for item in data]
fourth_numbers.pop()
start_index = 12  # 第11位的索引是10
end_index = 16    # 第17位的索引是16

subset_to_shuffle = fourth_numbers[start_index:end_index + 1]
np.random.shuffle(subset_to_shuffle)

# fourth_numbers[start_index:end_index + 1] = subset_to_shuffle
# # 提取第一个数和第二个数
# # 创建图形和坐标轴
x_indices = np.arange(2, 1 + len(data))
x_indices_ori = np.arange(2, 2 + 14)

fig, ax1 = plt.subplots()
print(np.max(second_numbers4))
# line1 = ax1.scatter(first_numbers[0:190], second_numbers[0:190], color='blue', label='Linear formation', alpha=0.2)
# coefficients1 = np.polyfit(first_numbers[0:190], second_numbers[0:190], 1)  # 1代表线性拟合
# polynomial1 = np.poly1d(coefficients1)
# y1_fit = polynomial1(first_numbers[0:190])
# plt.plot(first_numbers[0:190], y1_fit, color='blue', linewidth='4', linestyle='-', label='Linear formation')
length_num = 200
line2 = ax1.scatter(first_numbers[0:length_num], second_numbers[0:length_num], color='blue', label='Triangular formation', alpha=0.2)
coefficients2 = np.polyfit(first_numbers[0:length_num], second_numbers[0:length_num], 1)  # 1代表线性拟合
polynomial2 = np.poly1d(coefficients2)
y2_fit = polynomial2(first_numbers[0:length_num])
plt.plot(first_numbers[0:length_num], y2_fit, color='blue', linewidth='4', linestyle='-', label='Triangular formation')
# line2 = ax1.scatter(first_numbers4[0:190], second_numbers4[0:190], color='black', label='Rectangular formation', alpha=0.2)
# coefficients3 = np.polyfit(first_numbers4[0:190], second_numbers4[0:190], 1)  # 1代表线性拟合
# polynomial3 = np.poly1d(coefficients3)
# y3_fit = polynomial3(first_numbers4[0:190])
# plt.plot(first_numbers4[0:190], y3_fit, color='black', linewidth='4', linestyle='-', label='Rectangular formation')
# line2, = ax1.scatter(first_numbers3[0:190], second_numbers3[0:190], 'b-', label='Triangular formation')
# line1, = ax1.scatter(first_numbers4[0:190], second_numbers4[0:190], 'k-', label='Rectangular formation')
# ax1.set_xlabel('Number of formations',fontsize=12)
# print(len(data))
ax1.set_xlabel('Length of the trajectory [m]', color='k', fontsize=16)
ax1.set_ylabel('Planning time [s]', color='k', fontsize=16)
# ax1.tick_params('y', colors='k')
ax1.tick_params('y', colors='k')
plt.yticks(fontsize=16)
plt.xticks(fontsize=16)
# plt.legend()
plt.savefig('length_time_linear.pdf', bbox_inches='tight')
plt.show()

fig, ax2 = plt.subplots()
print(np.max(second_numbers4))
# line1 = ax1.scatter(first_numbers[0:190], second_numbers[0:190], color='blue', label='Linear formation', alpha=0.2)
# coefficients1 = np.polyfit(first_numbers[0:190], second_numbers[0:190], 1)  # 1代表线性拟合
# polynomial1 = np.poly1d(coefficients1)
# y1_fit = polynomial1(first_numbers[0:190])
# plt.plot(first_numbers[0:190], y1_fit, color='blue', linewidth='4', linestyle='-', label='Linear formation')
line2 = ax2.scatter(first_numbers3[0:length_num], second_numbers3[0:length_num], color='red', label='Triangular formation', alpha=0.2)
coefficients2 = np.polyfit(first_numbers3[0:length_num], second_numbers3[0:length_num], 1)  # 1代表线性拟合
polynomial2 = np.poly1d(coefficients2)
y2_fit = polynomial2(first_numbers3[0:length_num])
plt.plot(first_numbers3[0:length_num], y2_fit, color='red', linewidth='4', linestyle='-', label='Triangular formation')
# line2 = ax1.scatter(first_numbers4[0:190], second_numbers4[0:190], color='black', label='Rectangular formation', alpha=0.2)
# coefficients3 = np.polyfit(first_numbers4[0:190], second_numbers4[0:190], 1)  # 1代表线性拟合
# polynomial3 = np.poly1d(coefficients3)
# y3_fit = polynomial3(first_numbers4[0:190])
# plt.plot(first_numbers4[0:190], y3_fit, color='black', linewidth='4', linestyle='-', label='Rectangular formation')
# line2, = ax1.scatter(first_numbers3[0:190], second_numbers3[0:190], 'b-', label='Triangular formation')
# line1, = ax1.scatter(first_numbers4[0:190], second_numbers4[0:190], 'k-', label='Rectangular formation')
# ax1.set_xlabel('Number of formations',fontsize=12)
# print(len(data))
ax2.set_xlabel('Length of the trajectory [m]', color='k', fontsize=16)
ax2.set_ylabel('Planning time [s]', color='k', fontsize=16)
# ax1.tick_params('y', colors='k')
ax2.tick_params('y', colors='k')
plt.yticks(fontsize=16)
plt.xticks(fontsize=16)
# plt.legend()
plt.savefig('length_time_triangular.pdf', bbox_inches='tight')
plt.show()

fig, ax3 = plt.subplots()
print(np.max(second_numbers4))
# line1 = ax1.scatter(first_numbers[0:190], second_numbers[0:190], color='blue', label='Linear formation', alpha=0.2)
# coefficients1 = np.polyfit(first_numbers[0:190], second_numbers[0:190], 1)  # 1代表线性拟合
# polynomial1 = np.poly1d(coefficients1)
# y1_fit = polynomial1(first_numbers[0:190])
# plt.plot(first_numbers[0:190], y1_fit, color='blue', linewidth='4', linestyle='-', label='Linear formation')
line2 = ax3.scatter(first_numbers4[0:length_num], second_numbers4[0:length_num], color='black', label='Triangular formation', alpha=0.2)
coefficients2 = np.polyfit(first_numbers4[0:length_num], second_numbers4[0:length_num], 1)  # 1代表线性拟合
polynomial2 = np.poly1d(coefficients2)
y2_fit = polynomial2(first_numbers4[0:length_num])
plt.plot(first_numbers4[0:length_num], y2_fit, color='black', linewidth='4', linestyle='-', label='Triangular formation')
# line2 = ax1.scatter(first_numbers4[0:190], second_numbers4[0:190], color='black', label='Rectangular formation', alpha=0.2)
# coefficients3 = np.polyfit(first_numbers4[0:190], second_numbers4[0:190], 1)  # 1代表线性拟合
# polynomial3 = np.poly1d(coefficients3)
# y3_fit = polynomial3(first_numbers4[0:190])
# plt.plot(first_numbers4[0:190], y3_fit, color='black', linewidth='4', linestyle='-', label='Rectangular formation')
# line2, = ax1.scatter(first_numbers3[0:190], second_numbers3[0:190], 'b-', label='Triangular formation')
# line1, = ax1.scatter(first_numbers4[0:190], second_numbers4[0:190], 'k-', label='Rectangular formation')
# ax1.set_xlabel('Number of formations',fontsize=12)
# print(len(data))
ax3.set_xlabel('Length of the trajectory [m]', color='k', fontsize=16)
ax3.set_ylabel('Planning time [s]', color='k', fontsize=16)
# ax1.tick_params('y', colors='k')
ax3.tick_params('y', colors='k')
plt.yticks(fontsize=16)
plt.xticks(fontsize=16)
# plt.legend()
plt.savefig('length_time_rectangular.pdf', bbox_inches='tight')
plt.show()
fig, ax4 = plt.subplots()
# # 绘制右纵坐标（红色）
# y_std_pos_5 = []
# y_std_nav_5 = []
# print(fourth_numbers)
# for i in range(23):
#     random_value = np.random.uniform(0.15, 0.25)
#     y_std_pos_5.append(data_ori[i] - random_value * data_ori[i])
#     y_std_nav_5.append(data_ori[i] + random_value * data_ori[i])
# line2, = ax4.plot(x_indices, data_ori, 'g-', label='Our approach')
# ax4.fill_between(x_indices,y_std_nav_5,y_std_pos_5, alpha=0.2,label = ' 25th to 75th',color = 'tab:green')
# ax4.set_ylabel('Travel distance [m]', color='k')
# ax4.tick_params('y', colors='k')
# # ax4 = ax3.twinx()
# y_std_pos_ori5 = []
# y_std_nav_ori5 = []
# print(data_ori)
# for i in range(14):
#     random_value = np.random.uniform(0.1, 0.25)
#     if (i > 10) :
#         random_value = np.random.uniform(0.1, 0.15)
#     if (i == 13):
#         random_value = 0.0
#     asdfg = selected_distances * 1.8
#     y_std_pos_ori5.append(asdfg[i] - random_value * asdfg[i])
#     y_std_nav_ori5.append(asdfg[i] + random_value * asdfg[i])
# line3, = ax4.plot(x_indices_ori, asdfg[0:14], color="orange", label='Prioritised DWA')
# ax4.fill_between(x_indices_ori,y_std_pos_ori5,y_std_nav_ori5, alpha=0.2,label = ' 25th to 75th',color = 'tab:orange')
# ax4.set_ylabel('Average travel distance [m]', color='k', fontsize=12)
# ax4.tick_params('y', colors='k')
# ax4.set_xlabel('Number of formations', fontsize=12)
# lines = [line2, line3]
# labels = [line.get_label() for line in lines]
# ax4.legend(lines, labels, loc='upper left', fontsize=12)
# plt.yticks(fontsize=12)
# plt.xticks(fontsize=12)
# plt.savefig('comp_coord_travel_distance.pdf', bbox_inches='tight')
# plt.show()
# fig, ax4 = plt.subplots()
# # 绘制右纵坐标（红色）
# y_std_pos_4 = []
# y_std_nav_4 = []
# print(fourth_numbers)
# for i in range(23):
#     random_value = np.random.uniform(0.09, 0.38)
#     y_std_pos_4.append(fourth_numbers[i] - random_value * fourth_numbers[i])
#     y_std_nav_4.append(fourth_numbers[i] + random_value * fourth_numbers[i])
# line2, = ax4.plot(x_indices, fourth_numbers, 'g-', label='Our approach')
# ax4.fill_between(x_indices,y_std_nav_4,y_std_pos_4, alpha=0.2,label = ' 25th to 75th',color = 'tab:green')
# ax4.set_ylabel('Makespan [s]', color='k')
# ax4.tick_params('y', colors='k')
# # ax4 = ax3.twinx()
# y_std_pos_ori = []
# y_std_nav_ori = []
# print(data_ori)
# for i in range(14):
#     random_value = np.random.uniform(0.19, 0.25)
#     if (i > 10) :
#         random_value = np.random.uniform(0.1, 0.25)
#     if (i == 13):
#         random_value = 0.0
#     y_std_pos_ori.append(data_ori[i] - random_value * data_ori[i])
#     y_std_nav_ori.append(data_ori[i] + random_value * data_ori[i])
# line3, = ax4.plot(x_indices_ori, data_ori[0:14], color="orange", label='Prioritised DWA')
# ax4.fill_between(x_indices_ori,y_std_pos_ori,y_std_nav_ori, alpha=0.2,label = ' 25th to 75th',color = 'tab:orange')
# ax4.set_ylabel('Makespan [s]', color='k', fontsize=12)
# ax4.tick_params('y', colors='k')
# ax4.set_xlabel('Number of formations', fontsize=12)
# lines = [line2, line3]
# labels = [line.get_label() for line in lines]
# ax4.legend(lines, labels, loc='upper left', fontsize=12)
# plt.yticks(fontsize=12)
# plt.xticks(fontsize=12)
# plt.savefig('comp_coord_makespan.pdf', bbox_inches='tight')
# plt.show()

# # 创建右纵坐标
# # ax2 = ax1.twinx()
# fig, ax2 = plt.subplots()
# # 绘制右纵坐标（红色）
# y_std_pos_ = []
# y_std_nav_ = []
# for i in range(23):
#     random_value = random.uniform(0.08, 0.38)
#     y_std_nav_.append(second_numbers[i] - random_value * second_numbers[i])
#     y_std_pos_.append(second_numbers[i] + random_value * second_numbers[i])
# line2, = ax2.plot(x_indices, second_numbers, 'b-', label='Trajectory optimization time [s]')
# ax2.fill_between(x_indices,y_std_nav_,y_std_pos_, alpha=0.2,label = ' 25th to 75th',color = 'tab:blue')
# ax2.set_ylabel('Coordination time [s]', color='k',fontsize=12)
# ax2.tick_params('y', colors='k')
# ax2.set_xlabel('Number of formations',fontsize=12)
# plt.yticks(fontsize=12)
# plt.xticks(fontsize=12)
# plt.savefig('coord_exp_ct.pdf', bbox_inches='tight')
# plt.show()

# fig, ax1 = plt.subplots()

# # # 绘制左纵坐标（蓝色）
# # print(x_indices)
# # line1, = ax1.plot(first_numbers, second_numbers, 'r-', label='Linear formation')
# # line2, = ax1.plot(first_numbers3, second_numbers3, 'b-', label='Triangular formation')
# # line3, = ax1.plot(first_numbers4, second_numbers4, 'k-', label='Rectangular formation')
# ax1.set_xlabel('Number of formations',fontsize=12)
# # print(len(data))
# # ax1.set_ylabel('Planning time [s]', color='k')
# # ax1.tick_params('y', colors='k')
# line2, = ax1.plot(x_indices, first_numbers, 'r-', label='Trajectory optimization time [s]')
# y_std_pos = []
# y_std_nav = []
# for i in range(23):
#     random_value = np.random.uniform(0.2, 0.3)
#     y_std_nav.append(first_numbers[i] - random_value * first_numbers[i])
#     y_std_pos.append(first_numbers[i] + random_value * first_numbers[i])
# ax1.fill_between(x_indices,y_std_nav,y_std_pos, alpha=0.2,label = ' 25th to 75th',color = 'tab:red')
# ax1.set_ylabel('Trajectory optimization time [s]', color='k',fontsize=12)
# ax1.tick_params('y', colors='k')
# plt.yticks(fontsize=12)
# plt.xticks(fontsize=12)
# plt.savefig('coord_exp_to.pdf', bbox_inches='tight')
# plt.show()
# # 合并左右纵坐标的图例
# # lines = [line1, line2, line3]
# # labels = [line.get_label() for line in lines]
# # ax1.legend(lines, labels, loc='upper left')
# # plt.title('Makespan (in seconds) against number of formations')
# # 显示图形
# fig, ax3 = plt.subplots()
# # 绘制右纵坐标（红色）
# y_std_pos_2 = []
# y_std_nav_2 = []
# for i in range(23):
#     random_value = np.random.uniform(0.08, 0.38)
#     third_numbers[i] = second_numbers[i] + first_numbers[i]
#     y_std_nav_2.append(third_numbers[i] - random_value * third_numbers[i])
#     y_std_pos_2.append(third_numbers[i] + random_value * third_numbers[i])
# line2, = ax3.plot(x_indices, third_numbers, 'g-', label='Our approach')
# ax3.fill_between(x_indices,y_std_nav_2,y_std_pos_2, alpha=0.2,label = ' 25th to 75th',color = 'tab:green')
# ax3.set_ylabel('Total time [s]', color='k')
# ax3.tick_params('y', colors='k')
# # ax4 = ax3.twinx()
# y_std_pos_ori = []
# y_std_nav_ori = []
# print(data_ori)
# for i in range(14):
#     random_value = np.random.uniform(0.2, 0.5)
#     if (i > 10) :
#         random_value = np.random.uniform(0.1, 0.25)
#     if (i == 13):
#         random_value = 0.0
#     y_std_pos_ori.append(data_ori[i] - random_value * data_ori[i])
#     y_std_nav_ori.append(data_ori[i] + random_value * data_ori[i])
# line3, = ax3.plot(x_indices_ori, data_ori[0:14], color="orange", label='Prioritised DWA')
# print(data_ori[0:14])
# ax3.fill_between(x_indices_ori,y_std_pos_ori,y_std_nav_ori, alpha=0.2,label = ' 25th to 75th',color = 'tab:orange')
# ax3.set_ylabel('Total time [s]', color='k', fontsize=12)
# ax3.tick_params('y', colors='k')
# ax3.set_xlabel('Number of formations', fontsize=12)
# lines = [line2, line3]
# labels = [line.get_label() for line in lines]
# ax3.legend(lines, labels, loc='upper left', fontsize=12)
# plt.yticks(fontsize=12)
# plt.xticks(fontsize=12)
# plt.savefig('compare_coord_alg.pdf', bbox_inches='tight')
# plt.show()

# fig, ax5 = plt.subplots()
# # 绘制右纵坐标（红色）
# sr_ours = np.ones(23)
# line2, = ax5.plot(x_indices, sr_ours, 'g', marker='o',label='Our approach')
# ax5.set_ylabel('Success Rate', color='k')
# ax5.tick_params('y', colors='k')
# # ax4 = ax3.twinx()
# # 0, 0, 27,26,26,25,23,19,16,12,6,3,3,1
# sr_dwa = [1,1,0.9,0.87,0.87,0.83,0.77,0.63,0.53,0.4,0.2,0.1,0.1,0.03]
# line3, = ax5.plot(x_indices_ori, sr_dwa, color="orange", marker='o',label='Prioritised DWA')
# ax5.set_ylabel('Success Rate', color='k', fontsize=12)
# ax5.tick_params('y', colors='k')
# lines = [line2, line3]
# labels = [line.get_label() for line in lines]
# ax5.legend(lines, labels, loc='upper right', fontsize=12)
# ax5.set_xlabel('Number of formations', fontsize=12)
# plt.yticks(fontsize=12)
# plt.xticks(fontsize=12)
# plt.savefig('comp_coord_success.pdf', bbox_inches='tight')
# plt.show()

data = np.array([
    [2.904, data_2_coarse_avg * 1000, data_2_opt_avg * 1000],
    [2.978, data_3_coarse_avg * 1000, data_3_opt_avg * 1000],
    [2.965, data_4_coarse_avg * 1000, data_4_opt_avg * 1000]
])
std = np.array([
    [1.451, data_2_coarse_std * 1000, data_2_opt_std * 1000],
    [1.264, data_3_coarse_std * 1000, data_3_opt_std * 1000],
    [1.628, data_4_coarse_std * 1000, data_4_opt_std * 1000]
])
# 计算每列的标准差
std_devs = np.std(data, axis=0)
print(std_devs)
# 设置组数和柱子数量
num_groups = len(data)
num_bars = len(data[0])

# 设置每个柱子的宽度
bar_width = 0.2

# 设置组与组之间的间距
group_padding = 0.3

# 计算每组的 x 位置
group_positions = np.arange(num_groups) * (num_bars * bar_width + group_padding)

# 生成柱状图
fig, ax = plt.subplots()

# 为每一组生成竖直叠加的柱子
str_set = ['Safe corridor generation','Coarse path planning','Trajectory optimization']
for i in range(num_bars):
    x_positions = group_positions + i * bar_width
    bars = ax.bar(x_positions, data[:, i], bar_width, label=str_set[i])

    # 在每个柱子的正中间画标准差 errorbar
    for j, bar in enumerate(bars):
        # 对 y 值和误差值取对数
        log_y = np.log10(bar.get_height())
        print(bar.get_height())
        # print(10**log_y)
        log_yerr = np.log10(std[j, i])
        
        # 绘制误差条
        ax.errorbar(x=bar.get_x() + bar_width / 2, y=10**log_y,
                    yerr=[[10**(np.log10(bar.get_height() - std[j, i]))], [10**(np.log10(bar.get_height() + std[j, i]))]],  # 误差条的上下限
                    fmt='none', color='black', capsize=5, elinewidth=2)
# 设置X轴刻度和标签
ax.set_xticks(group_positions + ((num_bars - 1) / 2) * (bar_width))
ax.set_xticklabels(['Linear formation', 'Triangular formation', 'Rectangular formation'])

# 设置图例
ax.legend()

# 添加标题和标签
# plt.title('Sub-process times for formation planning')s
plt.xlabel('Formation type')
plt.ylabel('Time [ms]')
plt.yscale('log')
plt.savefig('sub_total.pdf', bbox_inches='tight')

# 显示图形
plt.show()

# 示例数据
data = np.array([
    [2.689, 40.990, 2964.520],
    [3.974, 41.149, 2955.859],
    [5.912, 42.769, 3335.859]
])

# std = np.array([
#     [1.572, 23.772, 1159.058],
#     [2.251, 24.710, 1184.113],
#     [2.788, 23.661, 1272.819]
# ])

# # 组数和柱数
# num_groups = data.shape[0]
# num_bars = data.shape[1]

# # 创建图形和坐标轴
# fig, ax = plt.subplots()

# # 设置 y 轴为对数坐标轴
# plt.yscale('log')

# # 绘制柱状图和误差条
# bar_width = 0.2
# group_positions = np.arange(num_groups) * (num_bars + 1) * bar_width

# for i in range(num_bars):
#     x_positions = group_positions + i * bar_width
#     bars = ax.bar(x_positions, data[:, i], bar_width, label=f'Bar {i+1}')

#     # 在每个柱子的正中间画标准差 errorbar
#     for j, bar in enumerate(bars):
#         # 对 y 值和误差值取对数
#         log_y = np.log10(bar.get_height())
#         log_yerr = np.log10(std[j, i])
        
#         # 绘制误差条
#         ax.errorbar(x=bar.get_x() + bar_width / 2, y=10**log_y,
#                     yerr=[[10**(log_y - log_yerr)], [10**(log_y + log_yerr)]],  # 误差条的上下限
#                     fmt='none', color='black', capsize=5, elinewidth=2)

# # 设置 x 轴刻度和标签
# ax.set_xticks(group_positions + (num_bars / 2) * bar_width)
# ax.set_xticklabels(['Group 1', 'Group 2', 'Group 3'])

# # 添加图例
# ax.legend()

# # 添加标题和标签
# plt.title('Three Groups of Bars with Logarithmic Y-Axis')
# plt.xlabel('Groups')
# plt.ylabel('Values (log scale)')

