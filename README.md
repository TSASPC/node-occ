# @tsaspc/node-occ

OpenCascade nodejs extension for solid modeling.

---- NOTICE ------
Module is being refactored for 7.5.0 Updates from OpenCASCADE Library

Current Work:

Rework Progress Indicators

Add GLTF Export

Add IGES Import/Export

Add Surface fitting support

Add NURBs support

Long Term goal:

Revamp to force XDE



This nodejs extension provides _solid construction_ to nodejs.
It provides a simple yet powerful javascript api to construct 3D geometry models.

This project comes with a set of V8 wrappers around OpenCascade API and a sample web application.

[![Build Status](https://travis-ci.org/OpenWebCAD/node-occ.png?branch=master)](https://travis-ci.org/OpenWebCAD/node-occ)
[![Build status](https://ci.appveyor.com/api/projects/status/s5eaux89v2c0wmu4?svg=true)](https://ci.appveyor.com/project/erossignon/node-occ-6ktv4)

### quick example

```javascript
var occ = require("@tsaspc/node-occ").occ;

// construct a box
var box = occ.makeBox([0, 0, 0], [100, 100, 50]);

// construct a cylinder
var cyl = occ.makeCylinder([50, 50, -10], [50, 50, 60], 40);

// cut the box with cylinder
box = occ.cut(box, cyl);

// save result to a STEP file
occ.writeSTEP("somefile.step", box);
```

### video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=swUPSa2zyrY" target="_blank"><img src="http://img.youtube.com/vi/swUPSa2zyrY/0.jpg" 
alt="node occ" width="240" height="180" border="10" /></a>

### list of features

-   creation of basic shapes ( box, cylinder , cone , torus )
-   boolean operation ( fuse , common , cut )
-   features ( draftAngle)
-   solid properties ( faces, edges, vertices, area , volume )
-   import export ( STEP BREP )
-   

### sample web application

[node-occ-sample](https://github.com/erossignon/node-occ-sample): sample nodejs/express REST API server to build solid , based on threejs

## installing node-occ from npm

```
$npm install @tsaspc/node-occ
```

## building node-occ from source : prerequisites


#### on (linux Ubuntu

(use nodejs 12 or 14)
# Prerequisites 
Node.JS

```bash
#CMake
sudo apt-get install cmake cmake-curses-gui g++ build-essential libtbb2
sudo apt-get install tcllib tklib tcl-dev tk-dev libfreetype6-dev libx11-dev libgl1-mesa-dev libfreeimage-dev rapidjson-dev
cmake -DCMAKE_SUPPRESS_REGENERATION:BOOL=ON -DOCE_MULTITHREADED_BUILD:BOOL=ON -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_TESTING:BOOLEAN=OFF -DOCE_DRAW:BOOLEAN=OFF -DOCE_TESTING:BOOLEAN=OFF -DOCE_OCAF:BOOLEAN=OFF -DOCE_VISUALISATION:BOOLEAN=OFF -DOCE_DISABLE_X11:BOOLEAN=ON -DUSE_RAPIDJSON:BOOLEAN=ON -DOCE_DISABLE_TKSERVICE_FONT:BOOLEAN=ON -DOCE_USE_PCH:BOOLEAN=ON .
sudo apt-get install libv8-dev
# ------------------------------------
git clone --recursive https://github.com/tsaspc/node-occ.git
cd node-occ

#
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
npm install --build-from-source

# verify that everything is working OK
make test
```

### On windows (Not Maintained)

-   follow the tutorial in the [wiki](https://github.com/erossignon/node-occ/wiki)

## Optional Dependencies:

-   threejs : https://github.com/mrdoob/three.js

## Acknowledgement:

-   OpenCascade : http://www.opencascade.org
-   occmodel : https://github.com/tenko/occmodel
-   ShapeSmith : https://github.com/bjnortier/shapesmith

## MIT License

Copyright © 2021 TSASPC LLC
Copyright © 2012-2019 E. Rossignon

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
