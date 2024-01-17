import React, { useState, useRef, useCallback } from 'react';
import { View, Text, TextInput, StyleSheet, PanResponder, useWindowDimensions, ScrollView, TouchableOpacity } from 'react-native';
import axios from 'axios';

const EspControlScreen = () => {
  const { width: screenWidth, height: screenHeight } = useWindowDimensions();

  const [espIpAddress, setEspIpAddress] = useState('172.20.10.4');
  const [espPort, setEspPort] = useState('80'); // Default port is 80
  const [response, setResponse] = useState(0);
  const [xValue, setXValue] = useState(0);
  const [yValue, setYValue] = useState(0);
  const [logs, setLogs] = useState([]);
  const [moveValue, setMoveValue] = useState('');
  const [rotateValue, setRotateValue] = useState('');


  const sendHttpRequest = useCallback(async (x, y) => {
    try {
      const url = `http://${espIpAddress}:${espPort}/your_endpoint`;
      const formData = new FormData();
      formData.append('data', `X:${x},Y:${y}`);
  
      const response = await axios.post(url, formData, {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
      });
      setResponse(response.data);
    } catch (error) {
      console.error('Error sending HTTP request:', error);
      addLog(`Error sending HTTP request: ${error.message}`);
    }
  }, [espIpAddress, espPort, setResponse, addLog]);
  
  const scrollViewRef = useRef();

  const addLog = (log) => {
    const timestamp = new Date().toLocaleTimeString();
    const newLog = `${timestamp} - ${log}`;
    setLogs((prevLogs) => {
      const newLogs = [...prevLogs, newLog];
      if (newLogs.length > 20) {
        newLogs.splice(0, newLogs.length - 20); // Keep only the last 20 logs
      }
      return newLogs;
    });
  };

  const handleMove = useCallback((_, gestureState) => {
    const containerWidth = screenWidth;
    const containerHeight = screenHeight;

    const centerX = containerWidth / 2;
    const centerY = containerHeight / 2;

    const xOffset = 3;
    const yOffset = 105;

    const displacementX = gestureState.moveX - centerX - xOffset;
    const displacementY = gestureState.moveY - centerY - yOffset;

    const newX = Math.max(-100, Math.min(100, displacementX));
    const newY = Math.max(-100, Math.min(100, displacementY));

    setXValue(Math.round(newX)); // Round to the nearest integer
    setYValue(Math.round(newY)); // Round to the nearest integer

    sendHttpRequest(Math.round(newX), Math.round(newY)); // Send HTTP request dynamically
  }, [screenWidth, screenHeight, espIpAddress, espPort]);

  const handleRelease = () => {
    setXValue(0);
    setYValue(0);
    sendHttpRequest(0, 0); // Send HTTP request with 0 values when released
  };

  const handleConfigUpdate = () => {
    // Perform any additional validation if needed
    // Update the IP address and port number
    console.log("New IP Address:", espIpAddress);
    console.log("New Port:", espPort);
  }

  const handleMoveCommand = async (moveValue) => {
    try {
      const url = `http://${espIpAddress}:${espPort}/move`;
      const formData = new FormData();
      formData.append('data', `move:${moveValue}cm`);
  
      const response = await axios.post(url, formData, {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
      });
      setResponse(response.data);
    } catch (error) {
      console.error('Error sending move command:', error);
      addLog(`Error sending move command: ${error.message}`);
    }
  };
  
  const handleRotateCommand = async (rotateValue) => {
    try {
      const url = `http://${espIpAddress}:${espPort}/rotate`;
      const formData = new FormData();
      formData.append('data', `rotate:${rotateValue}deg`);
  
      const response = await axios.post(url, formData, {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
      });
      setResponse(response.data);
    } catch (error) {
      console.error('Error sending rotate command:', error);
      addLog(`Error sending rotate command: ${error.message}`);
    }
  };  
  
  const panResponder = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onMoveShouldSetPanResponder: () => true,
      onPanResponderMove: handleMove,
      onPanResponderRelease: handleRelease,
    })
  ).current;

  return (
    <View style={styles.container}>
      <Text style={styles.title}>Dirty Bastards Control Screen</Text>

      <TextInput
        style={styles.input}
        placeholder="Enter ESP IP Address"
        onChangeText={(text) => setEspIpAddress(text)}
        value={espIpAddress}
      />

      <TextInput
        style={styles.input}
        placeholder="Enter Port (default: 8080)"
        onChangeText={(text) => setEspPort(text)}
        value={espPort}
      />

      <TextInput
        style={styles.input}
        placeholder="Move Value (cm)"
        onChangeText={(text) => setMoveValue(text)}
        value={moveValue}
      />

      <TouchableOpacity onPress={() => handleMoveCommand(moveValue)} style={styles.button}>
        <Text style={styles.buttonText}>Move</Text>
      </TouchableOpacity>

      <TextInput
        style={styles.input}
        placeholder="Rotate Value (degrees)"
        onChangeText={(text) => setRotateValue(text)}
        value={rotateValue}
      />

      <TouchableOpacity onPress={() => handleRotateCommand(rotateValue)} style={styles.button}>
        <Text style={styles.buttonText}>Rotate</Text>
      </TouchableOpacity>


      <TouchableOpacity onPress={handleConfigUpdate} style={styles.button}>
        <Text style={styles.buttonText}>Update Config</Text>
      </TouchableOpacity>

      <View style={styles.joystickContainer} {...panResponder.panHandlers}>
        <View style={[styles.joystickIndicator, { transform: [{ translateX: xValue }, { translateY: yValue }] }]} />
        <Text style={styles.joystickValues}>X: {xValue}, Y: {yValue}</Text>
      </View>

      <Text style={styles.responseText}>Response: {response}</Text>

      <ScrollView
        ref={scrollViewRef}
        style={styles.logContainer}
        onContentSizeChange={() => scrollViewRef.current.scrollToEnd({ animated: true })}
      >
        {logs.map((log, index) => (
          <Text key={index} style={styles.logText}>
            {log}
          </Text>
        ))}
      </ScrollView>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 16,
  },
  title: {
    fontSize: 20,
    marginBottom: 16,
  },
  input: {
    height: 40,
    borderColor: 'gray',
    borderWidth: 1,
    marginVertical: 8,
    padding: 5,
    width: '100%',
  },
  button: {
    backgroundColor: '#007BFF',
    padding: 10,
    borderRadius: 5,
    marginTop: 16,
  },
  buttonText: {
    color: 'white',
    textAlign: 'center',
  },
  responseText: {
    marginTop: 16,
  },
  joystickContainer: {
    position: 'relative',
    width: 200,
    height: 200,
    backgroundColor: 'lightgray',
    justifyContent: 'center',
    alignItems: 'center',
  },
  joystickIndicator: {
    position: 'absolute',
    width: 20,
    height: 20,
    backgroundColor: 'red',
    borderRadius: 10,
  },
  joystickValues: {
    marginTop: 8,
  },
  logContainer: {
    marginTop: 16,
    maxHeight: 100,
    borderColor: 'gray',
    borderWidth: 1,
    padding: 8,
  },
  logText: {
    fontSize: 12,
  },
  button: {
    backgroundColor: '#007BFF',
    padding: 10,
    borderRadius: 5,
    marginTop: 16,
  },
  buttonText: {
    color: 'white',
    textAlign: 'center',
  },
});

export default EspControlScreen;
