
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 获取当前文件夹下的 Excel 文件路径
current_dir = os.path.dirname(os.path.abspath(__file__))
file_name = "exp_data.xlsx"
file_path = os.path.join(current_dir, file_name)

# 读取 Excel 文件
df = pd.read_excel(file_path, sheet_name="sce1grid")

# 重命名列，去掉空格
df.columns = ["agent1", "agent2", "delay_departure"]

# 分离 0 左右的数据
df_left = df[df["delay_departure"] < 0]   # 负值部分
df_right = df[df["delay_departure"] >= 0]  # 非负值部分


# 创建折线图
plt.figure(figsize=(8, 5))

# 绘制 0 左侧部分（不与右侧相连）
plt.plot(df_left["delay_departure"], df_left["agent1"], label="Agent 1", marker='o', linestyle='-', color="blue")
plt.plot(df_left["delay_departure"], df_left["agent2"], label="Agent 2", marker='s', linestyle='-', color="orange")

# 绘制 0 右侧部分（不与左侧相连）
plt.plot(df_right["delay_departure"], df_right["agent1"], marker='o', linestyle='-', color="blue")
plt.plot(df_right["delay_departure"], df_right["agent2"], marker='s', linestyle='-', color="orange")

# 计算平均值
avg_agent2_left = df_left["agent2"].mean()
avg_agent1_right = df_right["agent1"].mean()

# 计算 x 轴范围
x_min, x_max = df["delay_departure"].min(), df["delay_departure"].max()

# 添加平均线
plt.axhline(y=avg_agent1_right, xmin=(-x_min)/(x_max-x_min), xmax=1, color="blue", linestyle="--", linewidth=1.5, label=f"Agent1 Avg Travel Time with Priority : {avg_agent1_right:.2f}s")
plt.axhline(y=avg_agent2_left, xmin=0, xmax=(-x_min+0.1)/(x_max-x_min), color="orange", linestyle="--", linewidth=1.5, label=f"Agent2 Avg Travel Time with Priority: {avg_agent2_left:.2f}s")

# 标注平均值
plt.text(df_left["delay_departure"].min(), avg_agent2_left, f"{avg_agent2_left:.2f}s", verticalalignment='bottom', fontsize=10)
plt.text(df_right["delay_departure"].max(), avg_agent1_right + 0.1, f"{avg_agent1_right:.2f}s", verticalalignment='bottom', fontsize=10)

# 添加 0 作为分界线
separation_line = plt.axvline(x=0, color='red', linestyle='-', linewidth=2, label="simultaneous entry")

# 添加标题和标签
plt.title("Travel Time under Grid-based Control")
plt.xlabel("Agent 2 Entry Time Relative to Agent 1 (s)")
plt.ylabel("Travel Time (s)")

# 获取已有图例元素，并手动添加分割线
handles, labels = plt.gca().get_legend_handles_labels()

# 显示图例
plt.legend(handles=handles, labels=labels)


# 显示图表
plt.grid(True)
plt.show()

avg_agent1 = df["agent1"].mean()
avg_agent2 = df["agent2"].mean()
print(avg_agent1, avg_agent2)