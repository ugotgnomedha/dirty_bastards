
document.getElementById('sendMoveDistance').addEventListener('click', function() {
  var move = document.getElementById("moveValue").value; // Use value instead of innerHTML for input element
  sendMoveDistance(move);
});

function sendMoveDistance(move) {
  var url = 'http://172.20.10.6/sendMoveDistance';
  var data = 'move=' + move + 'cm';
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
  var url = 'http://172.20.10.6/sendLidarDistance';
  var data = 'distance=' + distance + 'cm';
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

// _____________________________________________ WORKS


  document.getElementById('sendCompass').addEventListener('click', function() {
    var degrees = document.getElementById("compassValue").value; // Use value instead of innerHTML for input element
    sendCompassValue(degrees);
  });


// SHOULD WORK____________________________--->

// Function to send rotation value to Arduino
function sendCompassValue(degrees) {

  var url = 'http://172.20.10.6/sendCompassValue';
  var data = 'rotate=' + degrees + 'deg';
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

// __________________________________SHOULD WORK



// GET - METHODS ___________________________________________________________________________

// Function to retrieve current compass value from Arduino
function getCompassValue() {
  const url = 'http://172.20.10.6/getCompassValue';

  const xhr = new XMLHttpRequest();
  xhr.open('GET', url, true);

  xhr.onload = function() {
    if (xhr.status >= 200 && xhr.status < 300) {
      // Parse and display the compass value
      const compassValue = parseFloat(xhr.responseText);
      document.getElementById('compassValue').value = isNaN(compassValue) ? '' : compassValue;
    } else {
      console.error('Error:', xhr.statusText);
    }
  };

  xhr.onerror = function() {
    console.error('Network error');
  };

  xhr.send();

    // Update the rotation of the pin based on the compass value
    const pin = document.getElementById('pin');
    pin.style.transform = `translateX(-50%) rotate(${compassValue}deg)`;
}


// Function to retrieve current LIDAR distance from Arduino
function getLidarDistance() {
  const url = 'http://172.20.10.6/getLidarDistance';

  const xhr = new XMLHttpRequest();
  xhr.open('GET', url, true);

  xhr.onload = function() {
    if (xhr.status >= 200 && xhr.status < 300) {
      // Parse and display the LIDAR distance
      const lidarDistance = parseFloat(xhr.responseText);
      document.getElementById('lidarDistance').value = isNaN(lidarDistance) ? '' : lidarDistance;
    } else {
      console.error('Error:', xhr.statusText);
    }
  };

  xhr.onerror = function() {
    console.error('Network error');
  };

  xhr.send();

    // Update the Lidar bar fill
    const lidarFill = document.getElementById('lidarFill');
    lidarFill.style.width = `${lidarValue / 150 * 100}%`;
}



