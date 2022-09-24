const nodeocc = require("..");
const occ = nodeocc.occ;

console.log('starting up')
occ.readSTEP("VAT_Test.step", function (err, shapes) {
    console.log(shapes)
    //console.log(err)
    occ.writeGLTF("VAT_Test.gltf",shapes)
    console.log('Done writing')
});
