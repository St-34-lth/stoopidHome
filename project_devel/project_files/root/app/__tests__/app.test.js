import { beforeAll, afterAll, test, expect } from 'vitest';
import { io as ioc } from 'socket.io-client';
import app from '../main.js'; // Adjust the path if necessary

const port = 80;
const URL = `http://localhost:${port}`;

let clientSocket;

beforeAll(() => {
  return new Promise((resolve, reject) => {
    clientSocket = ioc(URL);

    clientSocket.on('connect', () => {
      console.log('Socket connected');
      resolve();
    });

    clientSocket.on('connect_error', (err) => {
      console.error('Connection error:', err);
      reject(err);
    });
  });
});

afterAll(() => {
  if (clientSocket && clientSocket.connected) {
    clientSocket.disconnect();
  }
});

test('test connection', async () => {
  expect(clientSocket.connected).toBe(true);

  return new Promise((resolve) => {
    clientSocket.on('connection_data', (data) => {
      console.log('Received connection_data:', data);
      expect(data).toBeDefined();
      resolve();
    });

    clientSocket.emit('request_data'); // Trigger data request if needed
  });
});

test('test connection_data', async () => {
  expect(clientSocket.connected).toBe(true);

  return new Promise((resolve) => {
    clientSocket.on('connection_data', (data) => {
      console.log('Received connection_data:', data);
      expect(data).toBeDefined();
      resolve();
    });

    clientSocket.emit('request_connection_data'); // Trigger data request if needed
  });
});
