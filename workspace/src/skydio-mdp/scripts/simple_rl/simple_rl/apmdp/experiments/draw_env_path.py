from simple_rl.apmdp.settings.build_cube_env_2 import build_cube_env
from simple_rl.apmdp.AMDP.LtlAMDPClass import LTLAMDP

import matplotlib.pyplot as plt
import os
from matplotlib import colors as pcolor
from mpl_toolkits.mplot3d import Axes3D as ax3d
import numpy as np


def draw_floor(ax, env, floor_num, color='black', alpha = 0.1):
    """ (x0, y0, z0) : left botton corner of the floor
        (nx, ny) :the number of grids
        (lx, ly) : the size of grids"""

    nx, ny, nz = env['len_x'], env['len_y'], env['len_z']  # the number of grids
    nfloor = env['num_floor']  # the number of floors

    x0, y0 = 1,1
    z0 = 1 + floor_num * (nz / nfloor)

    X, Y = np.meshgrid([x0, x0+ nx], [y0, y0+ ny])
    Z = z0*np.ones(X.shape)

    ax.plot_surface(X=X, Y=Y, Z=Z, facecolors=color, alpha=alpha)

    return ax

def draw_cubic(ax, x0, y0, z0, lx, ly, lz, facecolors='b', alpha=0.2):
    sval = {}

    X, Y = np.meshgrid([x0, x0+lx], [y0, y0+ly])
    sval[0] = {'X':X, 'Y': Y, 'Z': z0*np.ones(X.shape)}
    sval[1] = {'X':X, 'Y': Y, 'Z': (z0+lz)*np.ones(X.shape)}

    X, Z = np.meshgrid([x0, x0 + lx], [z0, z0 + lz])
    sval[2] = {'X': X, 'Y': y0*np.ones(X.shape), 'Z': Z}
    sval[3] = {'X': X, 'Y': (y0 + ly) * np.ones(X.shape), 'Z': Z}

    Y, Z = np.meshgrid([y0, y0 + ly], [z0, z0 + lz])
    sval[4] = {'X': x0 * np.ones(Y.shape), 'Y': Y, 'Z': Z}
    sval[5] = {'X': (x0 + lx) * np.ones(Y.shape), 'Y': Y, 'Z': Z}

    for ii in range(0, 6):
        ax.plot_surface(X=sval[ii]['X'], Y=sval[ii]['Y'], Z=sval[ii]['Z'], facecolors=facecolors, alpha=alpha)

def draw_room(ax, env, room_num, facecolors='blue', alpha= 0.3):
    room_locs = env['room_to_locs'][room_num]
    loc_origin = min(room_locs)
    draw_cubic(ax, loc_origin[0], loc_origin[1], loc_origin[2],
               env['room_len'], env['room_len'], env['room_height'],
               facecolors=facecolors, alpha=alpha)


if __name__ == "__main__":

    # Environment
    env = build_cube_env()

    # Solve problems
    init_loc = (1,1,1)
    ltl_formula = 'F(c & F d)'  # ex) 'F(a & F( b & Fc))', 'F a', '~a U b'
    ap_maps = {'a':[2, 'state', 3], 'b':[1,'state',2], 'c':[2, 'state', 5], 'd':[1, 'state', 8]}
    hlevel = 1

    ltl_amdp = LTLAMDP(ltl_formula, ap_maps, env_file=[env], slip_prob=0.0, verbose=True)

    sseq, aseq, len_actions, backup = ltl_amdp.solve(init_loc, FLAG_LOWEST=False)

    # make the prettier output
    s_seq, a_seq, r_seq, f_seq = ltl_amdp.format_output(sseq, aseq)


    # extract information
    room_lx, room_ly, room_lz = env['room_len'], env['room_len'], env['room_height']

    # Draw
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # draw floors
    if hlevel >= 2:
        f_seq_unique = []
        for floor_num in f_seq:
            if floor_num not in f_seq_unique:
                f_seq_unique.append(floor_num)
                draw_floor(ax, env, floor_num)


    # draw rooms
    r_seq_unique = []
    for room_num in r_seq:
        if room_num not in r_seq_unique:
            r_seq_unique.append(room_num)
            draw_room(ax, env, room_num, alpha=0.1)

    # draw paths
    xseq, yseq, zseq = [s[0]+0.5 for s in s_seq], [s[1]+0.5 for s in s_seq], [s[2]+0.5 for s in s_seq]

    for loc in s_seq:
        draw_cubic(ax, loc[0], loc[1], loc[2], lx=1, ly=1, lz=1, facecolors='r', alpha=0.2)

    # start and end point
    loc = s_seq[0]
    draw_cubic(ax, loc[0]+0.25, loc[1]+0.25, loc[2]+0.25, lx=0.5, ly=0.5, lz=0.5, facecolors='g', alpha=0.2)

    ax.plot(xseq, yseq, zseq, color='r', linewidth=5)
    ax.view_init(elev=30, azim=-56)

    plt.show(block=False)

    # save figure name
    fig_dir = '/media/ys/DATA/Research/amdp_ltl/results/'
    ii = 1
    while(1):
        fname = "{}path{}.png".format(fig_dir, ii)
        if os.path.exists(fname):
            ii = ii+1
        else:
            plt.savefig(fname)#, bbox_inches='tight')
            break


