
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 获取当前文件夹下的 Excel 文件路径
current_dir = os.path.dirname(os.path.abspath(__file__))
file_name = "exp_data.xlsx"
file_path = os.path.join(current_dir, file_name)

# 读取 Excel 文件
df = pd.read_excel(file_path, sheet_name="sce1vscs")

# 重命名列，去掉空格
df.columns = ["agent1", "agent2", "delay_departure"]

# 创建折线图
plt.figure(figsize=(8, 5))

# 绘制
plt.plot(df["delay_departure"], df["agent1"], label="Agent 1", marker='o', linestyle='-', color="blue")
plt.plot(df["delay_departure"], df["agent2"], label="Agent 2", marker='s', linestyle='-', color="orange")

# 计算平均值
avg_agent1 = df["agent1"].mean()
avg_agent2 = df["agent2"].mean()

# 添加平均线
plt.axhline(y=avg_agent1, color="blue", linestyle="--", linewidth=1.5, label=f"Avg Agent 1 Travel Time: {avg_agent1:.2f}s")
plt.axhline(y=avg_agent2, color="orange", linestyle="--", linewidth=1.5, label=f"Avg Agent 2 Travel Time: {avg_agent2:.2f}s")

# 标注平均值
plt.text(df["delay_departure"].min(), avg_agent1, f"{avg_agent1:.2f}s",  verticalalignment='bottom', fontsize=10)
plt.text(df["delay_departure"].max(), avg_agent2, f"{avg_agent2:.2f}s",  verticalalignment='bottom', fontsize=10)

# 添加标题和标签
plt.title("Travel Time under VSCS")
plt.xlabel("Agent 2 Entry Time Relative to Agent 1 (s)")
plt.ylabel("Travel Time (s)")
# 获取已有图例元素，并手动添加分割线
handles, labels = plt.gca().get_legend_handles_labels()

# 显示图例
plt.legend(handles=handles, labels=labels)
plt.grid(True)

# 显示图表
plt.show()