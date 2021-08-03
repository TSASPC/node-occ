const nodeocc = require("..");
const occ = nodeocc.occ;
occ.readSTEP("VAT_Test.step", function (err, shapes) {
    occ.writeGLTF("VAT_Test.gltf",shapes)
});
