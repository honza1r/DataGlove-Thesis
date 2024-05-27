# Use the official Nginx image as the base
FROM nginx:alpine

# Install Node.js and npm
RUN apk add --no-cache nodejs npm

# Create a directory for roslib
WORKDIR /usr/share/nginx/roslib

# Install roslib using npm
RUN npm install roslib

# Link roslib.min.js to the html directory
RUN ln -s /usr/share/nginx/roslib/node_modules/roslib/build/roslib.min.js /usr/share/nginx/html/roslib.min.js

# Remove the default Nginx configuration file
RUN rm /etc/nginx/conf.d/default.conf

# Copy a new configuration file from the current directory
COPY ./src/nginx.conf /etc/nginx/conf.d

# Copy the content of the web-content directory to the Nginx server
COPY ./src/web-content /usr/share/nginx/html

# Expose port 80 for the web server
EXPOSE 80
