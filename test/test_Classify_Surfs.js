const nodeocc = require("..");
const occ = nodeocc.occ;
occ.readSTEP("VAT_Test.step", function (err, shapes) {
    //Plane, Cylinder, Cone, Sphere, Torus, BezierSurface, BSplineSurface, SurfaceOfRevolution, SurfaceOfExtrusion, OtherSurface.
    for (let i = 0; i < shapes.length; i++){
        const solid = shapes[i];
        const faces = solid.getFaces()
        for (let j = 0; j < faces.length; j++){
            const face = faces[j];
            console.log(face.getType())
            console.log(face.getTypeJSON())
        }
    }
});
