import csv
import pandas as pd
import plotly.graph_objects as go

file_path = "./data/torque_log.csv"
cleaned_file = './data/torque_log_cleaned_nopandas.csv'

# 清理資料：移除每行最後兩欄
with open(file_path, 'r', newline='', encoding='utf-8') as infile, \
     open(cleaned_file, 'w', newline='', encoding='utf-8') as outfile:

    reader = csv.reader(infile)
    writer = csv.writer(outfile)
    header = next(reader)
    writer.writerow(header)

    for row in reader:
        writer.writerow(row[:-2])

# 讀取清理後的資料
data = pd.read_csv(cleaned_file, engine="python")
data['time'] = data['time'] - data['time'].iloc[0]

# 指定要畫的關節
joints_to_plot = [
    'l_hip_yaw','l_hip_roll','l_hip_pitch','l_knee','l_ank_pitch','l_ank_roll',
    'r_hip_yaw','r_hip_roll','r_hip_pitch','r_knee','r_ank_pitch','r_ank_roll'
]

# 建立互動式圖表
fig = go.Figure()

for joint in joints_to_plot:
    fig.add_trace(go.Scatter(
        x=data['time'],
        y=data[joint],
        mode='lines',
        name=joint
    ))

# 設定圖表格式
fig.update_layout(
    title="Torque Variation Over Time",
    xaxis_title="Time (s)",
    yaxis_title="Torque (Nm)",
    legend_title="Joint",
    hovermode='x unified',
    template='plotly_white',
    width=1200,
    height=600
)

fig.show()

# import csv
# import pandas as pd
# import matplotlib.pyplot as plt

# file_path = "./torque_log.csv"
# cleaned_file = './torque_log_cleaned_nopandas.csv'

# with open(file_path, 'r', newline='', encoding='utf-8') as infile, \
#      open(cleaned_file, 'w', newline='', encoding='utf-8') as outfile:

#     reader = csv.reader(infile)
#     writer = csv.writer(outfile)
#     header = next(reader)
#     writer.writerow(header)

#     for row in reader:
#         writer.writerow(row[:-2])

# data = pd.read_csv(cleaned_file, engine="python")
# data['time'] = data['time'] - data['time'].iloc[0]
# print(data)

# plt.figure(figsize=(15, 8))
# joints_to_plot = ['l_hip_yaw','l_hip_roll','l_hip_pitch','l_knee','l_ank_pitch','l_ank_roll',
#                   'r_hip_yaw','r_hip_roll','r_hip_pitch','r_knee','r_ank_pitch','r_ank_roll']   # 指定繪製關節
# # joints_to_plot = ['l_sho_pitch','l_sho_roll','l_el','r_sho_pitch','r_sho_roll','r_el','head_pan','head_tilt'] 

# for joint in joints_to_plot:
#     plt.plot(data['time'], data[joint], label=joint)
# # for joint in data.columns[1:]: 
# #     plt.plot(data['time'], abs(data[joint]), label=joint)

# plt.title("Torque Variation Over Time", fontsize=16)
# plt.xlabel("Time (s)", fontsize=14)
# plt.ylabel("Torque (Nm)", fontsize=14)
# plt.legend(loc="upper right", fontsize=10)
# plt.grid(True)

# plt.show()