import os
import matplotlib.pyplot as plt
import numpy as np

""" Read a result file and save contents"""
def read_file(filename):
    file = open(filename, "r")
    contents = file.readlines()
    file.close()
    return contents

if __name__ == "__main__":

    # Settings
    dir_files = "{}/results/".format(os.getcwd())
    dir_files = "/media/ys/DATA/Research/amdp_ltl/results/"
    dir_figure = "/media/ys/DATA/Research/amdp_ltl/results/"
    filenames = ['result_time_random_env1_more.txt', 'result_time_random_env1_level1.txt',
                 'result_time_random_env2_more.txt', 'result_time_random_env2_level1.txt']
    env_set = [1, 1, 2, 2]
    lowest_set = [0, 1, 0, 1]

    # initialize
    num_file = len(filenames)   # the number of result files
    Ndata = 100 # the number of cases

    # read contents from files
    contents_set={}
    for ii in range(0, num_file):
        contents_set[ii] = read_file(dir_files + filenames[ii])

    # parsing
    analysis = {}
    for num_env in range(0, num_file):
        contents = contents_set[num_env]
        var_names = contents[2].split(', ')

        # initialize lists (extract name of variables)
        clist={}
        for jj in range(0, 13):
            clist[var_names[jj]] = []
        clist['ap_maps']=[]

        # Read data
        for ii in range(3, len(contents)):
            cur_list = contents[ii].split(', ')
            if not(cur_list[6]=='-1' and cur_list[7]=='-1' and cur_list[8]=='-1') \
                    and not(cur_list[6]=='0' and cur_list[7]=='0' and cur_list[8]=='0'):
                for jj in range(0, 12):
                    clist[var_names[jj]].append(float(cur_list[jj]))
                clist['LTL'].append(cur_list[12])
                clist['ap_maps'].append(eval(', '.join(cur_list[13:])))

                if len(clist['time_AMDP']) == Ndata:
                    break

        # The number of successful cases
        print(np.sum(clist['correct_AMDP']), np.sum(clist['correct_pMDP']))
        print(np.sum([1 for ii in range(0, Ndata) if clist['len_AMDP'][ii]==clist['len_pMDP'][ii]]))
        # analysis
        ratio_time = np.divide(clist['time_AMDP'], clist['time_pMDP'])
        ratio_backup = np.divide(clist['backup_AMDP'], clist['backup_pMDP'])

        # histogram-ratio
        bin_edges_ratio = [ii*0.1 for ii in range(0, 18)]
        hist_time, bin_edges = np.histogram(ratio_time, bin_edges_ratio)
        hist_backup, _ = np.histogram(ratio_backup, bin_edges_ratio)

        # histogram -time
        nbin = np.linspace(0, max(max(clist['time_AMDP']), max(clist['time_pMDP'])), 20)
        hist_time_AMDP, bin_edges_time_AMDP = np.histogram(clist['time_AMDP'], nbin)
        hist_time_pMDP, bin_edges_time_pMDP = np.histogram(clist['time_pMDP'], bin_edges_time_AMDP)

        # histogram -backup
        nbin = np.linspace(0, max(max(clist['backup_AMDP']), max(clist['backup_pMDP'])), 20)
        hist_backup_AMDP, bin_edges_backup_AMDP = np.histogram(clist['backup_AMDP'], nbin)
        hist_backup_pMDP, bin_edges_backup_pMDP = np.histogram(clist['backup_pMDP'], nbin)

        analysis[num_env] = {'hist_time_ratio': hist_time, 'hist_backup_ratio': hist_backup, 'bin_edges_ratio': bin_edges_ratio,
                             'hist_time_AMDP': hist_time_AMDP, 'hist_time_pMDP': hist_time_pMDP,
                             'bin_edges_time_AMDP': bin_edges_time_AMDP, 'bin_edges_time_pMDP': bin_edges_time_pMDP,
                             'hist_backup_AMDP': hist_backup_AMDP, 'hist_backup_pMDP': hist_backup_pMDP,
                             'bin_edges_backup_AMDP': bin_edges_backup_AMDP, 'bin_edges_backup_pMDP': bin_edges_backup_pMDP}

    # time ratio histogram
    ftsize = 14
    f1 = plt.figure()
    _, ax = plt.subplots()
    mset = ['o', '^', '+', 's']
    cset = ['tab:blue', 'tab:blue', 'tab:red', 'tab:red']
    for ii in range(0,4):
        y = np.cumsum(analysis[ii]['hist_time_ratio'])
        plt.plot(bin_edges[1:], y, marker=mset[ii], linewidth=2, markersize=7)
        print("The number of cases where time ratio <=1: {}".format(y[np.where(bin_edges[1:]==1)[0]]))
    plt.grid(True)
    ax.legend(['$\mathcal{E}_1$, $\mathcal{l}=0$', '$\mathcal{E}_1$, $\mathcal{l}=1$',
                '$\mathcal{E}_2$, $\mathcal{l}=0$','$\mathcal{E}_2$, $\mathcal{l}=1$'], fontsize=ftsize)
    ax.set_xlabel('Computing time ratio of AP-MDP to P-MDP', fontsize=ftsize)
    ax.set_ylabel('Cumulative number of cases', fontsize=ftsize)
    ax.tick_params(labelsize='large')
    plt.show(block=False)
    plt.savefig("{}ratio_time2.png".format(dir_figure), bbox_inches='tight')


    # backup ratio histogram
    f2 = plt.figure()
    _, ax = plt.subplots()
    mset = ['o', '^', '+', 's']
    for ii in range(0, 4):
        y = np.cumsum(analysis[ii]['hist_backup_ratio'])
        ax.plot(bin_edges[1:], y, marker=mset[ii], linewidth=2, markersize=7)
        print("The number of cases where backup ratio <=1: {}".format(y[np.where(bin_edges[1:] == 1)[0]]))

    ax.legend(['$\mathcal{E}_1$, $\mathcal{l}=0$', '$\mathcal{E}_1$, $\mathcal{l}=1$',
                '$\mathcal{E}_2$, $\mathcal{l}=0$','$\mathcal{E}_2$, $\mathcal{l}=1$'], fontsize=ftsize)
    ax.set_xlabel('Backup ratio of AP-MDP to P-MDP', fontsize=ftsize)
    ax.set_ylabel('Cumulative number of cases', fontsize=ftsize)
    ax.grid(True)
    ax.tick_params(labelsize='large')

    plt.show(block=False)
    plt.savefig("{}ratio_backup2.png".format(dir_figure), bbox_inches='tight')



    # figure 3
    f3 = plt.figure()
    _, ax1 = plt.subplots()
    color = 'tab:red'
    ax1.plot(analysis[1]['bin_edges_time_AMDP'][1:], np.cumsum(analysis[1]['hist_time_AMDP']), marker='o', color=color)
    ax1.plot(analysis[1]['bin_edges_time_pMDP'][1:], np.cumsum(analysis[1]['hist_time_pMDP']), linestyle='--', marker='^', color=color)
    ax1.set_xlabel('Time (s)', fontsize=ftsize, color=color)
    ax1.set_ylabel('Cumulative number of cases', fontsize=ftsize)
    ax1.tick_params(labelsize='large')
    ax1.legend(['Time (AP-MDP)', 'Time (P-MDP)'], fontsize=ftsize-1, loc=(0.52, 0.27))

    ax2 = ax1.twiny()
    color = 'tab:blue'
    ax2.plot(analysis[1]['bin_edges_backup_AMDP'][1:], np.cumsum(analysis[1]['hist_backup_AMDP']), marker='o', color=color)
    ax2.plot(analysis[1]['bin_edges_backup_pMDP'][1:], np.cumsum(analysis[1]['hist_backup_pMDP']), linestyle='--', marker='^', color=color)
    ax2.set_xlabel('The number of backups', fontsize=ftsize, color=color)
    ax2.tick_params(labelsize='large')
    ax2.legend(['Backups (AP-MDP)', 'Backups (P-MDP)'], fontsize=ftsize-1, loc=(0.52, 0.1))


    plt.grid(True, alpha=0.5)
    plt.savefig("{}time_hist_small.png".format(dir_figure), bbox_inches='tight')
    plt.show(block=False)

    #
    f4 = plt.figure()
    _, ax1 = plt.subplots()
    color = 'tab:red'
    ax1.plot(analysis[3]['bin_edges_time_AMDP'][1:], np.cumsum(analysis[3]['hist_time_AMDP']), marker='o', color=color)
    ax1.plot(analysis[3]['bin_edges_time_pMDP'][1:], np.cumsum(analysis[3]['hist_time_pMDP']), linestyle='--', marker='^', color=color)
    ax1.set_xlabel('Time (s)', fontsize=ftsize, color=color)
    ax1.set_ylabel('Cumulative number of cases', fontsize=ftsize)
    ax1.tick_params(labelsize='large')
    ax1.legend(['Time (AP-MDP)', 'Time (P-MDP)'], fontsize=ftsize-1, loc=(0.52, 0.27))

    ax2 = ax1.twiny()
    color = 'tab:blue'
    ax2.plot(analysis[3]['bin_edges_backup_AMDP'][1:], np.cumsum(analysis[3]['hist_backup_AMDP']), marker='o', color=color)
    ax2.plot(analysis[3]['bin_edges_backup_pMDP'][1:], np.cumsum(analysis[3]['hist_backup_pMDP']), linestyle='--', marker='^', color=color)
    ax2.set_xlabel('The number of backups', fontsize=ftsize, color=color)
    ax2.tick_params(labelsize='large')
    ax2.legend(['Backups (AP-MDP)', 'Backups (P-MDP)'], fontsize=ftsize-1, loc=(0.52, 0.1))


    plt.grid(True, alpha=0.5)

    plt.savefig("{}time_hist_big.png".format(dir_figure), bbox_inches='tight')
    plt.show(block=False)


    print("end")