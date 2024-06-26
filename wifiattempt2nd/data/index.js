const endpoint = 'http://10.5.1.82';


document.getElementById('sendMoveDistance').addEventListener('click', function() {
  var move = document.getElementById("moveValue").value; // Use value instead of innerHTML for input element
  sendMoveDistance(move);
});


function sendMoveDistance(move) {
  var url = endpoint + '/sendMoveDistance';
  var data = 'move=' + move;
  url = url+'?'+data;

  var xhttp = new XMLHttpRequest();

  xhttp.open('POST', url, true);
  xhttp.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');

  xhttp.onload = function() {
      if (xhttp.status >= 200 && xhttp.status < 300) {
          console.log('Response:', xhttp.responseText);
      } else {
          console.error('Error:', xhttp.statusText);
      }
  };

  xhttp.onerror = function() {
      console.error('Network error');
  };

  xhttp.send();
}


// Function to send LIDAR distance to Arduino
document.getElementById('sendLidar').addEventListener('click', function() {
  var distance = document.getElementById("lidarDistance").value; // Use value instead of innerHTML for input element
  sendLidarDistance(distance);
});

// THIS WORKS ______________________________________ ->

function sendLidarDistance(distance) {
  var url = endpoint + '/sendLidarDistance';
  var data = 'distance=' + distance;
  url = url+'?'+data;

  var xhttp = new XMLHttpRequest();

  xhttp.open('POST', url, true);
  xhttp.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');

  xhttp.onload = function() {
      if (xhttp.status >= 200 && xhttp.status < 300) {
          console.log('Response:', xhttp.responseText);
      } else {
          console.error('Error:', xhttp.statusText);
      }
  };

  xhttp.onerror = function() {
      console.error('Network error');
  };

  xhttp.send();

  // const lidarFill = document.getElementById('lidarFill');
  // lidarFill.style.width = `${distance / 150 * 100}%`;
}

// _____________________________________________ WORKS


  document.getElementById('sendCompass').addEventListener('click', function() {
    var degrees = document.getElementById("compassValue").value; // Use value instead of innerHTML for input element
    sendCompassValue(degrees);
  });


// WORKS ____________________________--->

// Function to send rotation value to Arduino
function sendCompassValue(degrees) {

  var url = endpoint + '/sendCompassValue';
  var data = 'degrees=' + degrees;
  url = url+'?'+data;

  var xhttp = new XMLHttpRequest();

  xhttp.open('POST', url, true);
  xhttp.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');

  xhttp.onload = function() {
      if (xhttp.status >= 200 && xhttp.status < 300) {
          console.log('Response:', xhttp.responseText);
      } else {
          console.error('Error:', xhttp.statusText);
      }
  };

  xhttp.onerror = function() {
      console.error('Network error');
  };

  xhttp.send();

  // const pin = document.getElementById('pin');
  //   pin.style.transform = `translateX(-50%) rotate(${degrees}deg)`;
}

// __________________________________ WORKS



// GET - METHODS ___________________________________________________________________________
// This is currently fetching as text, would be better as a json to extract the values but this works
// also, the only problem is sometimes flooding the serial2 and glitching the website.



document.getElementById('get-read').addEventListener('click', function() {
  getSensorData();
});
// Function to retrieve current LIDAR distance and COMPASS degrees from Arduino

function getSensorData() {
  var url = endpoint + '/getSensorData';

  var xhttp = new XMLHttpRequest();

  xhttp.open('GET', url, true);

  xhttp.onload = function() {
    if (xhttp.status == 200) {
      var response = this.response;

      document.getElementById('readings-text').innerText = response;

      let str = response;
      let regex = /\d+/g;
      let matches = str.match(regex);

      // Check if matches array has at least two elements
      if (matches && matches.length >= 2) {
        let centimeters = parseInt(matches[0]);
        let degrees = parseInt(matches[1]);

        console.log("Lidar reading:", centimeters);
        console.log("Compass value:", degrees);



        // Update UI elements based on the extracted integer values
        const pin = document.getElementById('pin');
        pin.style.transform = `translateX(-50%) rotate(${degrees}deg)`; // Update compass rotation

        const lidarFill = document.getElementById('lidarFill');
        lidarFill.style.width = `${centimeters / 150 * 100}%`; // Update lidar fill width

      } else {
        console.error("Could not extract integer values from the response.");
      }
    } else {
      console.error('Error:', xhttp.statusText);
    }
  };

  xhttp.onerror = function() {
    console.error('Network error');
  };

  xhttp.send();

}

document.getElementById("drive").addEventListener('click',function(){
  startDriving();
});

function startDriving() {
  var url = endpoint + '/startDriving';
    var command = 'drive'; // Change this to your desired command

    var xhttp = new XMLHttpRequest();

    xhttp.open('POST', url, true);
    xhttp.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');

    xhttp.onload = function() {
        if (xhttp.status >= 200 && xhttp.status < 300) {
            console.log('Response:', xhttp.responseText);
        } else {
            console.error('Error:', xhttp.statusText);
        }
    };

    xhttp.onerror = function() {
        console.error('Network error');
    };

    // Send the command to Arduino
    xhttp.send('command=' + command);
}


document.getElementById("stop").addEventListener('click',function(){
  stopDriving();
});

function stopDriving() {
    var url = endpoint + '/stopDriving';
    var command = 'stop'; // Change this to your desired command

    var xhttp = new XMLHttpRequest();

    xhttp.open('POST', url, true);
    xhttp.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');

    xhttp.onload = function() {
        if (xhttp.status >= 200 && xhttp.status < 300) {
            console.log('Response:', xhttp.responseText);
        } else {
            console.error('Error:', xhttp.statusText);
        }
    };

    xhttp.onerror = function() {
        console.error('Network error');
    };

    // Send the command to Arduino
    xhttp.send('command=' + command);
}

// setInterval(getAccelerationData,500);

// function getAccelerationData() {
//   var url = endpoint + '/getAccelerationData';

//   var xhttp = new XMLHttpRequest();

//   xhttp.open('GET', url, true);

//   xhttp.onload = function() {
//     if (xhttp.status == 200) {
//       var response = this.response;
//       document.getElementById('acc_reading').innerText = response;

//      } else {
//       console.error('Error:', xhttp.statusText);
//     }
//   };

//   xhttp.onerror = function() {
//     console.error('Network error');
//   };

//   xhttp.send();

// }

document.getElementById('get-pulses').addEventListener('click', function() {
  getPulseData();
});
// Function to retrieve current LIDAR distance and COMPASS degrees from Arduino

function getPulseData() {
  var url = endpoint + '/getPulses';

  var xhttp = new XMLHttpRequest();

  xhttp.open('GET', url, true);

  xhttp.onload = function() {
    if (xhttp.status == 200) {
      var response = this.response;

      document.getElementById('pulses-text').innerText = response;
      console.log(response);

    } else {
      console.error('Error:', xhttp.statusText);
    }
  };

  xhttp.onerror = function() {
    console.error('Network error');
  };

  xhttp.send();

}