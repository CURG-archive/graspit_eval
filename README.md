## GraspIt! Eval
---

In the plugin directory or run ``source set_env.sh``
```
git submodule init
git submodule update # this will pull down the mongo driver
```

Build Mongo driver:

```
cd mongo-cxx-driver
scons --dbg=on --c++11 --ssl --sharedclient=SHAREDCLIENT -j4 install
```
Make sure to include c++11 in CONFIG and FLAGS in .pro (NEITHER OR NONE AT ALL)

Some dependency info:
```
sudo apt-get install dh-autoreconf
```
in Graspit, run 

```
./bin/graspit -p libthenameoftheplugin
```

or 

```
~/graspit/bin/graspit -p libgraspGenerationPlugin -c mug.iv
```

### Qt Creator Configuration

Build directory:

```
/home/timchunght/graspit_eval
```
Make arguments: ``-j5``

Run Executable:
```
/abs_path_to_graspit/graspit/bin/graspit
```

Run Arg: ``-p libgraspEvalPlugin``

### Introduction
------------
This plugin serves as an introduction to writing plugins for the GraspIt! grasping simulator.
It should be used as an example and to test your system configuration.

Disclaimer
----------
This is code that has been, and is currently being used, for research. There 
are still many unfinished pieces and plenty of ways to crash the system.  It's
by no means bullet proof. Please see the Introduction in the User Manual for 
more details.



Plugin Contests
---------------
helloWorldPlugin.pro
helloWorldPlugin.h
helloWorldPlugin.cpp

To compile, run qmake helloWorldPlugin.pro to generate a build file, then compile the build file. 



Troubleshooting
---------------
Ensure that you have compiled the plugin and GraspIt! using the same configuration mode [Release/Debug] or you will 
have segmentation errors due to a mismatch between the standard libraries on Windows.

On Windows, if your library uses the graspitGui GraspItGui class singleton to access the ivmanager or world, make sure to link against $(GRASPIT)/bin/graspit.lib. On Linux, there is no need to link against GraspIt!.

If you change the name of your graspit binary in your compilation environment, the name of the lib file will also change, and you will need to
recompile your plugin.

