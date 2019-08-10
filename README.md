# mujoco-py-to-c-viewer
Tempomary solution to https://github.com/openai/mujoco-py/issues/373

Exports joits to file and than the C-code uses them to go throught the simulation in mujoco-py. Currently fixing problem with simulation of objects with skin in MuJoCo-py.

How to use:
1. Modify your model (copy), so it does not use skin.
2. Use your python code with the model without skin to generate the sim.data.qpos (and than export it to csv by file.writerow(sim.data.qpos).
3. Use the csv with qpos + model with skin as inputs for the viewer (.cpp). Now you should be able to view the output of the simulation even with the skin.

There is also space for improvement, it does export active forces, so you could display them, but I did not bother as it was not importatnt for my use.
