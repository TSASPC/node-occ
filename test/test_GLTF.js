const nodeocc = require("..");
const occ = nodeocc.occ;
occ.readSTEP("battleship.STEP", function (err, shapes) {
    occ.writeGLTF("battleship.gltf",shapes)
});
