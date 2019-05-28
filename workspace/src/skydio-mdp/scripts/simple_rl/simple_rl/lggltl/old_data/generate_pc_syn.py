import sys

newsrcp, newtarp = 'hard_pc_src_syn.txt', 'hard_pc_tar_syn.txt'
core_colors = ['red', 'blue', 'green', 'yellow']
colors = ['red', 'blue', 'green', 'yellow', 'orange', 'pink', 'purple', 'violet']
c_map = {'red': 'R', 'pink': 'R', 'orange': 'R',
         'blue': 'B', 'purple': 'B', 'violet': 'B',
         'green': 'C',
         'yellow': 'Y'}


def main(srcp, tarp):
    with open(srcp, 'rb') as srcf, open(tarp, 'rb') as tarf:
        srcl, tarl = map(lambda x: x.strip(), srcf.readlines()), map(lambda x: x.strip(), tarf.readlines())
        assert len(srcl) == len(tarl)
        zl = zip(srcl, tarl)

    newsrcs, newtars = [], []
    for srcs, tars in zl:
        if 'X' in tars or 'Z' in tars:
            continue

        for srcw in srcs.split():
            if srcw in colors:
                for nc in filter(lambda x: c_map[x] != c_map[srcw], core_colors):
                    newsrc = srcs.replace(srcw, nc)
                    newtar = tars.replace(c_map[srcw], c_map[nc])
                    if newtar.count(c_map[nc]) == 1 and newtar != tars:
                        newsrcs.append(newsrc)
                        newtars.append(newtar)

    assert len(newsrcs) == len(newtars)
    for x, y in zip(newsrcs, newtars):
        print '{0}\t{1}'.format(x, y)

    print 'Generated {0} new synthetic samples and {1} new unique task types'.format(len(newsrcs), len(set(newtars)))
    with open(newsrcp, 'wb') as srcf, open(newtarp, 'wb') as tarf:
        srcf.write('\n'.join(srcl + newsrcs))
        tarf.write('\n'.join(tarl + newtars))


if __name__ == '__main__':

    if len(sys.argv) != 3:
        print 'Usage: python generate_pc_syn.py <src_data_file> <tar_data_file>'
        sys.exit(0)

    main(sys.argv[1], sys.argv[2])
