import csv
import pandas as pd
import matplotlib.pyplot as plt

file_path = "./torque_log.csv"
cleaned_file = './torque_log_cleaned_nopandas.csv'

with open(file_path, 'r', newline='', encoding='utf-8') as infile, \
     open(cleaned_file, 'w', newline='', encoding='utf-8') as outfile:

    reader = csv.reader(infile)
    writer = csv.writer(outfile)
    header = next(reader)
    writer.writerow(header)


    for row in reader:
        writer.writerow(row[:-2])

data = pd.read_csv(cleaned_file, engine="python")
data['time'] = data['time'] - data['time'].iloc[0]
print(data)

plt.figure(figsize=(15, 8))
joints_to_plot = ['r_knee']   # 指定繪製關節
for joint in joints_to_plot:
    plt.plot(data['time'], data[joint], label=joint)
# for joint in data.columns[1:]: 
#     plt.plot(data['time'], abs(data[joint]), label=joint)

plt.title("Torque Variation Over Time", fontsize=16)
plt.xlabel("Time (s)", fontsize=14)
plt.ylabel("Torque (Nm)", fontsize=14)
plt.legend(loc="upper right", fontsize=10)
plt.grid(True)

plt.show()