import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import re

ROOT_DIR = Path(__file__).resolve().parent
FILENAME = "actions.log"
FILE = ROOT_DIR.joinpath(FILENAME)
SORTED_FILE = ROOT_DIR.joinpath("sorted_actions.txt")

def sort_out_actions(file_in: Path, file_out: Path):
    """When actions simply start with ["""
    fi = open(file_in, "r")
    fo = open(file_out, "w")

    for line in fi.readlines():
        if line[0] == "[":
            fo.writelines(line)

    fi.close()
    fo.close()

def load_actions_to_numpy(file):
    
    with open(file, "r") as f:
        lines = f.readlines()

    n_lines = len(lines)
    actions = np.full((n_lines, 2), fill_value=0.0)
    for i in range(n_lines):
        # print(lines[i])
        t = re.findall(r"\[(.*?)\]",lines[i])
        splited = t[0].split(" ")
        numbers = [float(n) for n in set(splited) if n !=""]
        actions[i, :] = numbers

    return actions




if __name__ == '__main__':
    # test = load_actions_to_numpy(SORTED_FILE)
    # print(test[1:10])
    # test = "[1.1, 2]"
    # print(np.array(test))
    # test = np.loadtxt(SORTED_FILE, np.ndarray)
    # print(type(test))
    # load_actions_to_numpy(SORTED_FILE)
    # print("hei")
    actions = load_actions_to_numpy(SORTED_FILE)
    bins = np.arange(-1, 1, 0.1)
    # ac1_hist = np.histogram(test[:,0], bins)
    # ac2_hist = np.histogram(test[:,1], bins)
    # t = np.arange()
    dir = actions[:,0] - actions[:,1]
    plt.hist(dir[0:500000])

    # plt.figure(1)
    # plt.hist(test[:,0], bins)
    # plt.figure(2)
    # plt.hist(test[:,1], bins)

    # plt.xticks(bins)
    # plt.grid(True)
    plt.show()
    # print(ac1_hist)


    # print(test[1:10])
    # print(type(test))
    


