# Use the official Node.js 16 image as a parent image
FROM node:16

# Set the working directory in the container
WORKDIR /usr/src/app

# Copy package.json to the container
COPY src/package.json package.json

# Install any dependencies including 'cors'
RUN npm install

# Copy the rest of your application's source code from your host to your image filesystem
COPY src/ .

# Make the container's port 3000 available to the outside world
EXPOSE 3000

# Define the command to run your app using CMD which defines your runtime
CMD ["node", "index.js"]
