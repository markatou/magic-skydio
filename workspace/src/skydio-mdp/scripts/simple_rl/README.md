# simple_rl
A simple framework for experimenting with Reinforcement Learning in Python.

There are loads of other great libraries out there for RL. The aim of this one is twofold:

1. Simplicity.
2. Reproducibility of results.

A brief tutorial for a slightly earlier version is available [here](http://cs.brown.edu/~dabel/blog/posts/simple_rl.html). As of version 0.77, the library should work with both Python 2 and Python 3. Please let me know if you find that is not the case!

simple_rl requires [numpy](http://www.numpy.org/) and [matplotlib](http://matplotlib.org/). Some MDPs have visuals, too, which requires [pygame](http://www.pygame.org/news). Also includes support for hooking into any of the [Open AI Gym environments](https://gym.openai.com/envs). I recently added a basic test script, contained in the _tests_ directory.

This branch is forked from https://github.com/abagaria/simple_rl
and includes a new package _AP-MDP_.
You can find the original _simple_rl_ and its tutorial [here](https://github.com/david-abel/simple_rl).
# AP-MDP
Abstract Product Markov Decision Process (AP-MDP) is a framework
that translates LTL into its corresponding automata, creates a product MDP 
of the LTL specification and the environment
MDP, and decomposes the problem into subproblems to enable
efficient planning with abstractions. 

Detail of this project is described in the following paper:

Yoonseon Oh, Roma Patel, Thao Nguyen, Baichuan Huang, Ellie Pavlick, Stefanie Tellex,
    "Planning with State Abstractions for Non-Markovian Task Specifications",
    Robotics: Science and Systems (RSS), 2019 (Accepted)` 


## installation
Download simple_rl:

    git clone https://github.com/yoonseon-oh/simple_rl.git

_AP-MDP_ requires [spot2](https://spot.lrde.epita.fr/). Please install the **Debian** packages.


## Overview
* AP-MDP: includes all components for the _AP-MDP_ algorithm. 
It follows the structure of _AMDP_ (simple_rl/simple_rl/amdp).

* experiments
    * run_experiments_random.py: executes simulations to compare AP-MDP and a plain product MDP.
    * run_experiments.py: includes functions to execute AP-MDP and comparable algorithms. 
    The function _run_apMDP()_ executes LTL commands using AP-MDP.
* settings : builds the 3D space consisting of open spaces and walls. It includes semantic information, connectivity between states, 


* results: contains results of simulations.

