# dirty_bastards
Dirsty idiots

## ESP commands:
### Move forward/backward:

#### Request:
```pressing a button on the web ->
function sendMoveDistance(move) {
  var url = 'http://172.20.10.6/sendMoveDistance';
  var data = 'move=' + move + 'cm';
  url = url+'?'+data;
```
#### Response:
```ESP Serial
"Move distance received: " + moveData
```
```Serial1
"Move command received successfully"
```

### Rotate:

#### Request:
```pressing a button on the web ->
function sendCompassValue(degrees) {
  const url = 'http://172.20.10.6/sendCompassValue';
  const data = 'rotate=' + degrees + 'deg';
  url = url+'?'+data;
```
#### Response:
```ESP Serial
"Compass value received: " + compassData
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


#### JS Console on the web:
Response: Rotate command received successfully
XHR finished loading: POST "http://172.20.10.6/sendCompassValue?rotate=30deg".
sendCompassValue @ index.js:102

Response: LIDAR distance received successfully
XHR finished loading: POST "http://172.20.10.6/sendLidarDistance?distance=40cm".
sendLidarDistance @ index.js:63

Response: Move command received successfully
XHR finished loading: POST "http://172.20.10.6/sendMoveDistance?move=20cm".
senMoveDistance @ index.js:29
