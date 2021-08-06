const nodeocc = require("..");
const occ = nodeocc.occ;
occ.readSTEP("Sphere.step", function (err, shapes) {
    //Plane, Cylinder, Cone, Sphere, Torus, BezierSurface, BSplineSurface, SurfaceOfRevolution, SurfaceOfExtrusion, OtherSurface.
    for (let i = 0; i < shapes.length; i++){
        const solid = shapes[i];
        const faces = solid.getFaces()
        for (let j = 0; j < faces.length; j++){
            const face = faces[j];
            console.log(face.getType())
            //console.log(face.getTypeJSON())
            const wires = face.getWires(); 
            for (let k = 0; k < wires.length; k++){
                const edges = wires[k].getEdges();
                console.log(k)
                for (let l = 0; l < edges.length; l++){
                    console.log(edges[l].getType())
                    console.log(edges[l].getTypeJSON())
                    
                }
            }
        }
    }
});
