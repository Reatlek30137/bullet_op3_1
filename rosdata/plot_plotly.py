import pandas as pd
import plotly.graph_objects as go

# 讀取資料
csv_path = "./rosdata/op3_torque_log.csv"  # 確保路徑正確
df = pd.read_csv(csv_path, parse_dates=["timestamp"])

# 將 joint_name 攤平成欄
pivot_df = df.pivot(index="timestamp", columns="joint_name", values="estimated_torque_Nm")

# 建立互動圖表
fig = go.Figure()
for joint in pivot_df.columns:
    fig.add_trace(go.Scatter(x=pivot_df.index, y=pivot_df[joint], mode='lines', name=joint))

fig.update_layout(
    title="Estimated Torque per Joint Over Time (Interactive)",
    xaxis_title="Time",
    yaxis_title="Torque (Nm)",
    hovermode="x unified",
    legend_title="Joint Name"
)

fig.show()
