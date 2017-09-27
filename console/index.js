const SerialPort = require('serialport');

const Readline = SerialPort.parsers.Readline;

var port = new SerialPort('/dev/ttyUSB0', {
  baudRate: 57600
});

const parser = port.pipe(new Readline({ delimiter: '\n' }));
// const parser = port.pipe(new Delimiter({ delimiter: Buffer.from('EOL') }));

const stdin = process.stdin;

stdin.setRawMode(true);
stdin.resume();
stdin.setEncoding( 'utf8' );

let connected = false;

let variables = {};

let currYaw = 100;
let currPitch = 100;
let currRoll = 100;

let desYaw = 100;
let desPitch = 100;
let desRoll = 100;

function printConsole() {
  console.log('\x1Bc');
  console.log("===============================");
  console.log("Welcome to console navigation");
  console.log("===============================");
  console.log("Commands");
  console.log("-------------------------------");
  console.log("c -> connect");
  console.log("");
  console.log("Status");
  console.log("-------------------------------");
  console.log("Connected ", connected);
  console.log("Throttle ", variables["throttle"]);
  console.log("");
  console.log("Desired Orientation");
  console.log("-------------------------------");
  console.log("Yaw ", variables["rxYaw"]);
  console.log("Pitch ", variables["rxPitch"]);
  console.log("Roll ", variables["rxRoll"]);
  console.log("");
  console.log("Current Orientation");
  console.log("-------------------------------");
  console.log("Yaw ", variables["yawAngle"]);
  console.log("Pitch ", variables["pitchAngle"]);
  console.log("Roll ", variables["rollAngle"]);
  console.log("");
  console.log("PID");
  console.log("-------------------------------");
  console.log("Left", variables["rxLeft"]);
  console.log("Right", variables["rxRight"]);
  console.log("Yaw", variables["pidYaw"]);
  console.log("Pitch", variables["pidPitch"]);
  console.log("Roll", variables["pidRoll"]);
  console.log("");
  console.log("Desired Orientation");
  console.log("-------------------------------");
  console.log("Motor_A ", variables["Motor_A"]);
  console.log("Motor_B ", variables["Motor_B"]);
  console.log("Motor_C ", variables["Motor_C"]);
  console.log("Motor_D ", variables["Motor_D"]);
  console.log("");
  console.log("-------------------------------");
  console.log("Thanks");
}

setInterval(printConsole, 100);

function processMessage (str) {
  const keyPair = str.split('|');

  if (connected) {
    for (var i = 0; i < keyPair.length; i++) {
      var elements = keyPair[i].split('=');
      if (elements.length == 2) {
        variables[elements[0]] = elements[1];        
      }
    }
  }
}

stdin.on( 'data', function( key ){
  // ctrl-c ( end of text )
  if ( key === '\u0003' ) {
    process.exit();
  }

  if (key === 'c') {
    connected = !connected;
  }

  // write the key to stdout all normal like
  // process.stdout.write( key );
});

port.open(function(){
  console.log('Serial Port Opened');
  connected  = true;
  parser.on('data', function(data) {
    processMessage(data);
  });
});