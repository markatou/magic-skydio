#!/bin/bash
sudo rm -rf simple_rl-0.8-py3.5.egg
sudo rm -rf dist/simple_rl-0.8-py3.5.egg
python3 setup.py build
sudo python3 setup.py install
sudo cp -r simple_rl/lggltl/data/* "/usr/local/lib/python3.5/dist-packages/simple_rl-0.8-py3.5.egg/simple_rl/lggltl/data"
sudo cp simple_rl/lggltl/models/torch/encoder "/usr/local/lib/python3.5/dist-packages/simple_rl-0.8-py3.5.egg/simple_rl/lggltl/models/torch"
sudo cp simple_rl/lggltl/models/torch/decoder "/usr/local/lib/python3.5/dist-packages/simple_rl-0.8-py3.5.egg/simple_rl/lggltl/models/torch"


