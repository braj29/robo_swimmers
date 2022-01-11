import matplotlib
from sys import platform
if platform == "darwin":
    matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from glob import glob
import pandas as pd
from mpl_toolkits import mplot3d
import numpy as np
import os
import plotly.offline as py
import plotly.graph_objs as go

# Parameters
path = "/home/maarten/CLionProjects/revolve/output/cpg_bo/main_1557701167/"
var1 = "range_ub"
var2 = "signal_factor"
parameter_file = "parameters.txt"

# Set matplotlib font
font = {'size' : 13}
matplotlib.rc('font', **font)

# Get all sub-directories
path_list = glob(path + "*")
path_list = [path_ for path_ in path_list if os.path.isdir(path_)]
n_dirs = len(path_list)

# Holder
results = np.empty((n_dirs, 3))

# Check if it's numeric:
for ix, path_ in enumerate(path_list):
    try:
        int(path_.split("/")[-1] )
    except:
        continue

    # Cope with unsorted issue
    path_ = "/".join(path_.split("/")[:-1]) + "/" + str(ix)
    print(path_, ix) # All fine till here

    # Get parameters for this file
    subfolder_list = glob(path_ + "/*/")
    parameters = []

    # Get some parameter file (so no empty list anymore)
    c = 0
    while parameters == []:
        parameters = [(line.rstrip('\n')) for line in open(path_ + "/" + subfolder_list[c].split("/")[-2] + "/" + parameter_file)]
        c += 1

    for param in parameters:
        if(var1 in param):
            results[ix, 0] = float(param.split(":")[-1])
        if(var2 in param):
            results[ix, 1] = float(param.split(":")[-1])

    # Get Fitness. This is an average file in path_
    fitness = ".".join(glob(path_ + "/*.png")[0].split("/")[-1].split(".")[:-1])
    #fitness = ".".join(glob(path_ + "/*d.png")[0].split("/")[-1].split(".")[:-1])
    results[ix, 2] = fitness[:-1]

# Combine results
df = pd.DataFrame(results)
results = df.sort_values(by = [0,1]).to_numpy()

# Prepare for 3D plot
my_size_x = 11
my_size_y = 13
X =results[:,0].reshape((my_size_x,  my_size_y))
Y =results[:,1].reshape((my_size_x,  my_size_y))
Z =results[:,2].reshape((my_size_x,  my_size_y))

# Construct 3D plot
fig = plt.figure(figsize=(10,10))
ax = plt.axes(projection='3d')
ax.plot_surface(X, Y, Z, rstride=1, cstride=1,
               cmap='viridis', edgecolor='none')
ax.set_xlabel(var1)
ax.set_ylabel(var2)
ax.set_zlabel("fitness")
plt.savefig(path + "3Dplot.png")


# Get data for plotly
data = [
    go.Surface(
        x = X,
        y = Y,
        z = Z
    )
]

# Layout
layout = go.Layout(
    title = path,
    scene = dict(
        xaxis=dict(title=var1),
        yaxis=dict(title=var2),
        zaxis=dict(title="fitness"),
    ),
)

# Construct ploty
fig = go.Figure(data=data, layout = layout)
py.plot(fig, filename=path + "interactive_3d")

# Save XYZ
np.savetxt(path + "X.txt", np.matrix(X), fmt='%f', delimiter = ",")
np.savetxt(path + "Y.txt", np.matrix(Y), fmt='%f', delimiter = ",")
np.savetxt(path + "Z.txt", np.matrix(Z), fmt='%f', delimiter = ",")

