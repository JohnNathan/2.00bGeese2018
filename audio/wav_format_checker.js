
function padOctet(num) {
    num = num.toString(2);
    return "0".repeat(8 - num.length) + num;
}

function logPair(key, val) {
    console.log("%s: %d", key, val);
}

function read4(buf, loc) {
    return buf.readInt32LE(loc);
}

function read2(buf, loc) {
    return buf.readInt16LE(loc);
}

function foo(err, data, fname) {
    if (err) return console.log(err);
    
    console.log("File Name: %s", fname);
    
    logPair("File Size", read4(data, 4));
    logPair("fmt chunk size", read4(data, 16));
    logPair("Format Code", read2(data, 20));
    logPair("# Channels", read2(data, 22));
    logPair("Sample Rate", read4(data, 24));
    logPair("Resolution", read2(data, 34));
    console.log();
}

var fs = require('fs');

for (var i = 2; i < process.argv.length; ++i) {
    console.log();
    var fname = process.argv[i];
    fs.readFile(fname, function(err, data) { return foo(err, data, fname); });
}