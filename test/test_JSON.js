
const nodeocc = require("..");
const occ = nodeocc.occ;

solid = occ.makeBox([0, 0, 0], [10, 20, 30]);
const faces = solid.faces;
console.log(solid)
for (const face in faces){
    console.log(faces[face].getTypeJSON())
}