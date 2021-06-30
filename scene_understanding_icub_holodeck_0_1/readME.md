Installation Guide
==============================================================
*Author*: **Tuğberk Oğulcan ÇAKICI** (ogulcan.cakici@tum.de)

### Installation

Navigate into ``` $HOME/.opt/nrpStorage/``` and clone this repository. 
Now you should have ```$HOME/.opt/nrpStorage/tumproject```


NRP Models
=============

### Structure

The models to be spawned are located in ``$HBP/Models`` where ``$HBP`` is 
set to ``$HOME/Documents/NRP``
as described in 
[this repo.](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform/src/master/)

If you want to add a new model you need to move your model folder into this directory. In
order for it to be compatible, add your model folder named with CamelCase characters and 
enumerate them starting from 0. If you have, for example, a kitchen
table set the folder name KitchenTable0 if you have more than one, 
name them as KitchenTable1, KitchenTable2 ...

Everytime adding a new model or changing an existing model navigate into ```$HBP/Models``` and 
run the command:
```shell script
./create-symlinks.sh
```
this will crate a link for the changes.


The similar items should be grouped together. KitchenTable0, Refrigerator0, Oven0, 
CoffeeMachine0 
should be grouped under ``$HBP/kitchen/`` and some items can also be further groped.
 For example, Cup0, Cup1, Bottle0, Bottle1 should be located in ``$HBP/kitchen/TableItems``. 
 
In the model folder, open the ``model.sdf`` file and change the paths if necessary. The path can be seen
under <visual> tag and should be in format as ``model://<new_name>/meshes/...`` where it is
pointing to visual.dae files. ```<collision>``` tag can be deleted from ``model.sdf`` file since we are not
interested in the movements and collision of the objects. In this case, ```<gravity>0</gravity>``` line
should be added to ``model.sdf`` file to make sure object is not free falling.

It is very likely that an object can not be directly seen in the UI rendering. In such scenario, the object
should be present at backend rendering. The following screenshot can be a good example:

![SS](https://github.com/togulcan/tumproject/blob/master/docs/ss.png)

Note that although the robot can see some models from backend rendering(right), 
we can not see anything in UI rendering(left).

The models are spawning based on a layout configuration named ``layout.yaml`` located at 
corresponding room folder in ``$HBP/Models`` such as kitchen file. An example of this layout can be as follow:

````yaml
Layout0:
  - folder: KitchenTable0
    position:
      x: -1.0
      y: 1.70
      z: 0.04
      ox: 0
      oy: 0
      oz: 90
  - folder: KitchenCabinet0
    position:
      x: -0.6
      y: -0.5
      z: 0.0
      ox: 0
      oy: 0
      oz: 180
  - folder: Trash0
    position:
      x: -1.0
      y: 1.70
      z: 0.0
      ox: 0
      oy: 0
      oz: 0
  - folder: Refrigerator0
    position:
      x: 0.4
      y: 1.7
      z: 0.0
      ox: 0
      oy: 0
      oz: 270
  - folder: TableItems  # Will select an item randomly
    has_subfolder: True # indicate we have more than one object in TableItems folder
    position:
      x: -1.417
      y: 1.70
      z: 0.851
  - folder: TableItems # Will select an item randomly 
    has_subfolder: True
    position:
      x: -1.217
      y: 1.70
      z: 0.851
  - folder: TableItems/Cup0 # Will spawn Cup0 under TableItems
    position:
      x: -1.017
      y: 1.70
      z: 0.851
````
 
 In this layout, ```x,y,z``` represent cartesian coordinate and ```ox,oy,oz``` represent euler
 angles for orientations. If these values are not present in layout file 
  then the script will look for the `configs.yaml` file under the related object directory. ``has_subfolder`` indicated the TableItems folder contains sub models
 that are going to be chosen randomly and spawned at the given coordinates. More than one Layout 
 configuration can also be defines as follow:
 
 ````yaml
Layout0:
  - folder: KitchenTable0
    position:
      x: -1.0
      y: 1.70
      z: 0.04
      ox: 0
      oy: 0
      oz: 90
  - folder: KitchenCabinet0
    position:
      x: -0.6
      y: -0.5
      z: 0.0
      ox: 0
      oy: 0
      oz: 180

Layout1:
  - folder: KitchenTable1
    position:
      x: 1.2
      y: -4.30
      z: 0.0
      ox: 0
      oy: 0
      oz: 90
  - folder: KitchenCabinet2
    position:
      x: 0.3
      y: 0.5
      z: 0.0
      ox: 0
      oy: 0
      oz: 0
````

### Spawning

Now start the gazebo simulation by running the commands
```shell script
$ cle-nginx
$ cle-start
```
open your web browser(http://localhost:9000/#/esv-private) and click Scan Storage
 button on the top. To spawn a whole room such as kitchen, bedroom, etc. 
open a terminal inside ```$HOME/.opt/nrpStorage/tumproject``` and use the following command:

```shell script
$ python spawn.py -r <roomname> -l <layoutNumber>
```

where the layout number is the layout you want to spawn in ```layout.yaml``` file. Each room folder contains layout.yaml files indicating the positioning of the objects.
An example to spawn whole kitchen objects with a layout can be: 

```shell script
$ python spawn.py -r kitchen -l 0
```

this will goes inside ```$HBP/Models/kitchen``` and recursively spawn the objects in Layout0 in layout.yaml

---If you want to spawn a single object you should use the following command:

```shell script
$ python spawn.py -o <objectName> -p <position>
```
where the position is the x, y, z position in a string such as "0,0,0". An example of a spawning ball object in ```$HBP/Models``` can be:

```shell script
$ python spawn.py -o ball -p "0,0,0"
```

if you want to spawn a Refrigerator0 object inside the kitchen folder you should use the following command:
```shell script
$ python spawn.py -o kitchen/Refrigerator0 -p "0,0,0"
```

Each object has a configuration file `configs.yaml` for orientation. 
You can see such an example at ```$HBP/Models/kitchen/Refrigerator0```

If you want to spawn an object with given orientation, you need to set `ox,oy,oz` in `layout.yaml` file.
Otherwise the `configs.yaml` file will be used for setting initial orientation.

After spawning a new object, if you want to change the positions and orientations of the objects you should use the APIs in model.py file. Navigate into $HOME/.opt/nrpStorage/tumproject and open a terminal and write $python to start python shell. I would definitely recommend using python interactive shells such as bpython and IPython. Use the following commands to change the position and orientation of the, lets say, Refrigerator0 object.
```python
from model import Adjuster
obj = Adjuster("Refrigerator0")
obj.get_object_pose() # it will give you the current position of the object
obj.change_pose(x=1) # this will set the x position to 1
obj.change_pose(ox=90) # this will set the x angle to 90 degree.
obj.change_pose(y=1.24) # set the y position to 1.24
obj.change_pose(x=2, ox=90, oy=180) # you can change more than one position at the same time
```

where again `x,y,z` are cartesian coordinates, and `ox,oy,oz` are the orientation coordinates between 0 and 360 degrees.