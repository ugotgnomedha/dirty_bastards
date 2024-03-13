# dirty_bastards
Dirsty idiots

## ESP commands:
### Move forward/backward:

#### Request:
```pressing a button on the web ->
function sendMoveDistance(distance) {
  var url = 'http://172.20.10.6/move';
  var data = 'move distance=' + distance + 'cm';
  url = url+'?'+data;
```
#### Response:
```Serial
"Move command received: " + moveData
```
```Serial1
"Move command received successfully"
```

### Rotate:

#### Request:
```pressing a button on the web ->
function sendRotateCompassValue(degrees) {
  const url = 'http://172.20.10.6/rotate';
  const data = 'rotate:' + degrees + 'deg';
  url = url+'?'+data;
```
#### Response:
```Serial
"Rotate to compass command received: " + rotateData
```
```Serial1
"Rotate command received successfully"
```

### Lidar stop distance:

#### Request:
```pressing a button on the web ->
function sendLidarDistance(distance) {
  var url = 'http://172.20.10.6/sendLidarDistance';
  var data = 'distance=' + distance + 'cm';
  url = url+'?'+data;
```
#### Response:
```Serial
"LIDAR distance received: " + lidarData
```
```Serial1
"LIDAR distance received successfully"
```
