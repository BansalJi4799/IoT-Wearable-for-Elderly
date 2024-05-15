const mqtt = require("mqtt");
const fs = require("fs");
const path = require("path");

// MQTT broker settings
const mqttBroker = "mqtt://broker.hivemq.com";
const mqttPort = 1883; // Default MQTT port
const mqttTopic = "iot/data";

let fileCounter = 0;
let currentFile = null;

// Connect to MQTT broker
const client = mqtt.connect(mqttBroker, { port: mqttPort });

// Subscribe to MQTT topic
client.on("connect", () => {
  console.log("Connected to MQTT broker");
  client.subscribe(mqttTopic);
  createNewFile(); // Create a new file when connected
});

// Handle incoming messages
client.on("message", (topic, message) => {
  const data = message.toString(); // Convert message buffer to string
  console.log(data);
  if (!currentFile) {
    createNewFile(); // Create a new file if not already created
  }
  saveToCSV(data);
});

// Handle disconnection
client.on("close", () => {
  console.log("Disconnected from MQTT broker");
  closeFile(); // Close the current file
});

// Handle errors
client.on("error", (error) => {
  console.error("MQTT error:", error);
});

// Function to create a new file
function createNewFile() {
  fileCounter++;
  const fileName = `data_${fileCounter}.csv`; // Create file name with sequence number
  const filePath = path.join(__dirname, fileName); // Construct file path

  // Create new file
  fs.writeFileSync(filePath, "Data\n"); // Write header to file

  currentFile = filePath;
  console.log("New file created:", fileName);
}

// Function to save data to CSV file
function saveToCSV(data) {
  if (currentFile) {
    // Append data to file
    fs.appendFileSync(currentFile, `${data}\n`, (err) => {
      if (err) {
        console.error("Error writing to file:", err);
      } else {
        console.log("Data saved to CSV file:", currentFile);
      }
    });
  }
}

// Function to close the current file
function closeFile() {
  currentFile = null;
  console.log("File closed");
}
