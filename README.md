# dirty_bastards
Dirsty idiots

## ESP commands:
### Move forward/backward:

#### Request:
```console
curl -X POST http://172.20.10.4/move -d "data=move:dist:10cm"
```
#### Responce:
```console
Move command received successfully
```

### Rotate:

#### Request:
```console
curl -X POST http://172.20.10.4/rotate -d "data=rotate:-40deg"
```
#### Responce:
```console
Rotate command received successfull
```
