const express = require('express');
const cors = require('cors');

const app = express();
const fs = require('fs');
const path = require('path');

// Use CORS middleware to enable CORS
app.use(cors());
// JSON middleware for parsing JSON bodies
app.use(express.json());

const PORT = process.env.PORT || 3000;

// Function to write configuration to the shared volume
const writeConfig = (newConfig) => {
  const configPath = path.join('/shared-data', 'micro_ros_config.json');
  
  // Read the current configuration
  let config = readCurrentConfig(); // Use the readCurrentConfig function defined previously

  // Update the configuration with new values preserving unchanged values
  config = { ...config, ...newConfig };

  // Write the updated configuration back to the file
  fs.writeFileSync(configPath, JSON.stringify(config, null, 2), 'utf8');
  console.log('Configuration file updated successfully');
};

const readCurrentConfig = () => {
  const configPath = path.join('/shared-data', 'micro_ros_config.json');
  let config = { port: "8888", transport: "udp4", started: "false" }; // Default configuration

  // Check if the configuration file exists
  if (fs.existsSync(configPath)) {
      // If it exists, read and parse the current configuration
      const configContent = fs.readFileSync(configPath, 'utf8');
      config = JSON.parse(configContent);
  } else {
      // If the file doesn't exist, write the default configuration to the file
      fs.writeFileSync(configPath, JSON.stringify(config, null, 2), 'utf8');
  }
  return config;
};

app.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});

app.post('/start_agent', (req, res) => {
  // Assuming the client sends both port and transport in the request body
  const { port, transport } = req.body;
  // Write the configuration including the "started" state, port, and transport
  writeConfig({ "port": port, "transport": transport, "started": "true" });
  res.json({ message: 'Micro-ROS agent start requested.' });
  console.log('Micro-ROS agent start requested.');
});

app.post('/stop_agent', (req, res) => {
  // Read the current configuration, toggle the "started" state to "false"
  const currentConfig = readCurrentConfig();
  currentConfig.started = "false";
  // Write the updated configuration
  writeConfig(currentConfig);
  res.json({ message: 'Micro-ROS agent stop requested.' });
  console.log('Micro-ROS agent stop requested.');
});


