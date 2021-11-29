import pandas as pd
import matplotlib.pyplot as plt
import subprocess

command = 'adb pull /data/voxl_mapper/benchmark.csv'
process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
output, error = process.communicate()

csv_file='benchmark.csv'
df = pd.read_csv(csv_file, sep=',')

versions = list(df['Version'].unique())

plt.ion()
plt.show()

for version in versions:
    curr_df = df[df['Version'] == version]

    timing_labels = list(curr_df.columns[4:8])
    general_labels = list(curr_df.columns[10:])

    curr_df["random_tot"] = ((curr_df["Random Node Time"] / curr_df["Total Time"])*100)
    curr_df["nearest_tot"] = ((curr_df["Nearest Node Time"] / curr_df["Total Time"])*100)
    curr_df["check_tot"] = ((curr_df["Check Motion Time"] / curr_df["Total Time"])*100)
    curr_df["neigh_tot"] = ((curr_df["Neighborhood Time"] / curr_df["Total Time"])*100)

    timing_avg = [curr_df['random_tot'].mean(), curr_df['nearest_tot'].mean(), curr_df['check_tot'].mean(), curr_df['neigh_tot'].mean()]

    curr_df["t_nearest_fails"] = ((curr_df["Nearest Fails"] / curr_df["Attempts"])*100)
    curr_df["t_random_nearest_fails"] = ((curr_df["Random to Nearest Fails"] / curr_df["Attempts"])*100)
    curr_df["t_partial_segments"] = ((curr_df["Partial Segments"] / curr_df["Attempts"])*100)
    curr_df["t_tree_rewires"] = ((curr_df["Tree Rewires"] / curr_df["Attempts"])*100)

    general_avg = [curr_df['t_nearest_fails'].mean(), curr_df['t_random_nearest_fails'].mean(), curr_df['t_partial_segments'].mean(), curr_df['t_tree_rewires'].mean()]

    fig, axs = plt.subplots(2,2)
    fig.suptitle('v' + version, fontsize=20)
    axs[0,0].pie(timing_avg,labels=timing_labels,autopct='%.2f%%', shadow=True)
    axs[0,0].title.set_text('Timing Stats vs Total Time')

    axs[0,1].pie(general_avg,labels=general_labels, autopct='%.2f%%', shadow=True)
    axs[0,1].title.set_text('Failures vs Total Attempts')


    distance = list(curr_df["Distance"])
    total_time = list(curr_df["Total Time"])
    attempts = list(curr_df["Attempts"])

    axs[1,1].scatter(distance,attempts)
    axs[1,1].title.set_text('Distance vs Attempts')

    axs[1,0].scatter(distance,total_time)
    axs[1,0].title.set_text('Distance vs Time')
    plt.gcf().canvas.draw()
    _ = input("Press [enter] to continue.")
