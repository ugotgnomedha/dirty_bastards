# dirty_bastards
Dirsty idiots

## ESP commands:
### Move forward/backward:

#### Request:
'''bash
curl -X POST http://172.20.10.4/move -d "data=move:dist:10cm"
'''
#### Responce:
'''bash
Move command received successfully
'''

### Rotate:

#### Request:
'''bash
curl -X POST http://172.20.10.4/rotate -d "data=rotate:-40deg"
'''
#### Responce:
'''bash
Rotate command received successfull
'''
