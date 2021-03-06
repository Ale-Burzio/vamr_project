# README

The screencasts were captured on:

Lenovo X1 Carbon (Kitti, Parking, Malaga, Countrylife)
11th Gen Intel(R) Core(TM) i7-1165G7 @ 2.80Ghz
32 GB RAM

MacBook Pro 2020 (Vineyards)
8th Gen Intel(R) Core(TM) i7 1.7GHz (Turbo Boost 4.5 GHz)
16 GB RAM

We are using the most recent version of Matlab (2021b).

# Run program

Change the selected dataset by modifying the ds variable in the getparameters.m config file, then run the PROJECT.m file as is (no extra arguments).
Plots can be toggled on or off using the plot_XXX flags in the configuration file (significant speedup without plotting).

## Folder structure

The folders should be organized like this:

```
vamr_project
|
| README.md
| report.pdf
|
|___	code
|	|____	continuous_operation
|	|		| CO_processframe.m
|	|		| C_F_angle.m
|	|
|	|____ initialization
|	|		| initialization_KLT.m
|	|		| plotCoordinateFrame.m
|	|		| arrow3d.m
|	|
|	| PROJECT.m
|	| getparameters.m
|
|___	datasets
	| 	kitti
	|	|__	...
	| 	parking
	|	|__ ...
	| 	malaga
		|__ ...
```

## Dependencies

* Matlab 2021b
* Image Processing Toolbox
* Computer Vision Toolbox
* Statistics and Machine Learning Toolbox

## Screencasts
[Kitti dataset](https://www.youtube.com/watch?v=lVYn6uOW2Vs)

[Malaga dataset](https://youtu.be/CiI_OeyePWU)

[Parking dataset](https://www.youtube.com/watch?v=EgOp20M0F6E)

[Country life dataset](https://www.youtube.com/watch?v=oIe0Q46ND0M)

[Vineyards dataset](https://youtu.be/etVDBDrH5zo)
