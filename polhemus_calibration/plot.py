import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def plotAbestCaseUnpluggedPlugged(db):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    xs = [x[0] for x in db['a']] # + db['b'] + db['c'] + db['d'] + db['e'] + db['f']]
    ys = [x[1] for x in db['a']] # + db['b'] + db['c'] + db['d'] + db['e'] + db['f']]
    zs = [x[2] for x in db['a']] # + db['b'] + db['c'] + db['d'] + db['e'] + db['f']]

    ax.scatter(xs, ys, zs, c='b', marker='o')

    s = "bc"
    x1s = [x[0] for x in db['a'+s]] # + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    y1s = [x[1] for x in db['a'+s]] #+ db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    z1s = [x[2] for x in db['a'+s]] # + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]

    ax.scatter(x1s, y1s, z1s, c='b', marker='^')   
            
    s = "wc_plugged"
    x2s = [x[0] for x in db['a'+s]] # + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    y2s = [x[1] for x in db['a'+s]] #+ db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    z2s = [x[2] for x in db['a'+s]] # + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    
    ax.scatter(x2s, y2s, z2s, c='r', marker='o')
    
    s = "wc_unplugged"
    x3s = [x[0] for x in db['a'+s]] # + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    y3s = [x[1] for x in db['a'+s]] #+ db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    z3s = [x[2] for x in db['a'+s]] # + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    
    ax.scatter(x3s, y3s, z3s, c='r', marker='^')
        
    plt.show()
    
    
def plotAllWithBestCase(db):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
 
    xs = [x[0] for x in db['a'] + db['b'] + db['c'] + db['d'] + db['e'] + db['f'] + db['g']]
    ys = [x[1] for x in db['a'] + db['b'] + db['c'] + db['d'] + db['e'] + db['f'] + db['g']]
    zs = [x[2] for x in db['a'] + db['b'] + db['c'] + db['d'] + db['e'] + db['f'] + db['g']]

    ax.scatter(xs, ys, zs, c='b', marker='o')

    s = "bc"
    x1s = [x[0] for x in db['a'+s] + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    y1s = [x[1] for x in db['a'+s] + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]
    z1s = [x[2] for x in db['a'+s] + db['b'+s] + db['c'+s] + db['d'+s] + db['e'+s] + db['f'+s]]

    ax.scatter(x1s, y1s, z1s, c='r', marker='^')

    plt.show()

