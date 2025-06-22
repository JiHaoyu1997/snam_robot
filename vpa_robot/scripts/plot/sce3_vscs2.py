import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 获取当前文件夹下的 Excel 文件路径
current_dir = os.path.dirname(os.path.abspath(__file__))
file_name = "exp_data.xlsx"
file_path = os.path.join(current_dir, file_name)

# 读取 Excel 文件
df = pd.read_excel(file_path, sheet_name="sce3vscs2")

# 只保留三列数据
df.columns = ["agent1", "agent2", "agent3"]

# 生成实验次数（从 1 开始）
experiment_index = range(1, len(df) + 1)

# 计算每个 Agent 的最小值及其索引
min_agent1 = df["agent1"].min()
min_agent1_idx = df["agent1"].idxmin() + 1  # 索引转换为实验编号

min_agent2 = df["agent2"].min()
min_agent2_idx = df["agent2"].idxmin() + 1  # 索引转换为实验编号

min_agent3 = df["agent3"].min()
min_agent3_idx = df["agent3"].idxmin() + 1  # 索引转换为实验编号

# 创建折线图
plt.figure(figsize=(8, 5))

# 绘制曲线
plt.plot(experiment_index, df["agent1"], label="Agent 1", marker='o', linestyle='-', color="blue")
plt.plot(experiment_index, df["agent2"], label="Agent 2", marker='s', linestyle='-', color="orange")
plt.plot(experiment_index, df["agent3"], label="Agent 3", marker='*', linestyle='-', color="green")

# 计算平均值
avg_agent1 = df["agent1"].mean()
avg_agent2 = df["agent2"].mean()
avg_agent3 = df["agent3"].mean()

# 添加平均线
plt.axhline(y=avg_agent1, color="blue", linestyle="--", linewidth=1.5, label=f"Avg Agent1 Travel Time: {avg_agent1:.2f}s")
plt.axhline(y=avg_agent2, color="orange", linestyle="--", linewidth=1.5, label=f"Avg Agent2 Travel Time: {avg_agent2:.2f}s")
plt.axhline(y=avg_agent3, color="green", linestyle="--", linewidth=1.5, label=f"Avg Agent3 Travel Time: {avg_agent3:.2f}s")

# 标注平均值
# plt.text(1, avg_agent1, f"{avg_agent1:.2f}s", verticalalignment='bottom', fontsize=10)
# plt.text(len(df)/2, avg_agent2, f"{avg_agent2:.2f}s", verticalalignment='bottom', fontsize=10)
# plt.text(len(df), avg_agent3, f"{avg_agent3:.2f}s", verticalalignment='bottom', fontsize=10)

# 标注最小值
plt.scatter(min_agent1_idx, min_agent1, color="blue", marker='o', s=80, edgecolors="black", zorder=3)
plt.text(min_agent1_idx, min_agent1-0.4, f"Min {min_agent1:.2f}s", verticalalignment='top', fontsize=10, color="blue")

plt.scatter(min_agent2_idx, min_agent2, color="orange", marker='s', s=80, edgecolors="black", zorder=3)
plt.text(min_agent2_idx, min_agent2-0.2, f"Min {min_agent2:.2f}s", verticalalignment='top', fontsize=10, color="orange")

plt.scatter(min_agent3_idx, min_agent3, color="green", marker='*', s=80, edgecolors="black", zorder=3)
plt.text(min_agent3_idx, min_agent3-0.2, f"Min {min_agent3:.2f}s", verticalalignment='top', fontsize=10, color="green")

# 添加标题和标签
plt.title("Travel Time under VSCS for a Larger Safety Distance")
plt.xlabel("Experiment Number")
plt.ylabel("Travel Time (s)")

# 显示图例
plt.legend(loc='upper left', framealpha=0.5) # 左上角
plt.grid(True)
# plt.ylim(2, 12)  # 设置 y 轴范围

# 显示图表
plt.show()
