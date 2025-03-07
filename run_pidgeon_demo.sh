#!/bin/bash
set -e  # Exit on error

# Create a function to clean up processes on exit
cleanup() {
    echo "Cleaning up..."
    
    if [ -n "$SERVER_PID" ]; then
        echo "Stopping pidgeoneer server..."
        kill -9 $SERVER_PID >/dev/null 2>&1 || true
    fi
    
    if [ -n "$CONTROLLER_PID" ]; then
        echo "Stopping PID controller..."
        kill -9 $CONTROLLER_PID >/dev/null 2>&1 || true
    fi
    
    echo "Cleanup complete!"
}

# Register the cleanup function to run on exit
trap cleanup EXIT

# Print banner
echo "
    ____  _     __                                     
   / __ \(_)___/ /_____ ____  ____  ___  ___  _____   
  / /_/ / / __  / / __ \/ __ \/ __ \/ _ \/ _ \/ ___/   
 / ____/ / /_/ / / /_/ / / / / /_/ /  __/  __/ /       
/_/   /_/\__,_/_/\__, /_/ /_/\____/\___/\___/_/        
                /____/                                 
"

echo "Starting Pidgeon PID Controller Demo..."
echo "This script will start all components needed for the demo:"
echo " - Pidgeoneer Leptos web server (port 3000)"
echo " - A demonstration PID controller"
echo ""
echo "Press Ctrl+C to stop all components"
echo "------------------------------------------------------"

# Step 1: Start the Pidgeoneer web server
echo "Starting Pidgeoneer web server on port 3000..."
cd crates/pidgeoneer

# Check if cargo-leptos is installed
if ! cargo leptos --version >/dev/null 2>&1; then
    echo "Error: cargo-leptos is not installed."
    echo "Please install it with: cargo install cargo-leptos"
    exit 1
fi

# Start the server - using nohup to properly detach the process
# Redirecting both stdout and stderr to log file
echo "Starting server with nohup..."
nohup cargo leptos watch > leptos_server.log 2>&1 &
SERVER_PID=$!

# Verify the process started
if ! ps -p $SERVER_PID > /dev/null; then
    echo "Error: Failed to start the server. Please check for errors."
    exit 1
fi

cd ../..
echo "Pidgeoneer web server started with PID: $SERVER_PID"
echo ""

# Give server time to start and check if it's still running
echo "Waiting for server to initialize..."
sleep 5
if ! ps -p $SERVER_PID > /dev/null; then
    echo "Error: Server process died. Please check for errors."
    exit 1
fi

# Check if the server is responding
echo "Checking if server is responding..."
if command -v curl >/dev/null 2>&1; then
    if ! curl -s --head --fail http://localhost:3000 >/dev/null 2>&1; then
        echo "Warning: Server may not be fully initialized yet, but continuing anyway..."
    else
        echo "Server is responding correctly."
    fi
fi

# Open the browser to the dashboard
echo "Opening dashboard in your browser..."
# Use open for macOS, xdg-open for Linux, or start for Windows
if [[ "$OSTYPE" == "darwin"* ]]; then
    open "http://localhost:3000"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    xdg-open "http://localhost:3000" &>/dev/null || true
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    start "http://localhost:3000" || true
else
    echo "Could not automatically open browser. Please manually navigate to http://localhost:3000"
fi

# Wait for user to press S before continuing
echo ""
echo "Dashboard is ready at http://localhost:3000"
echo -n "Press 'S' to start the PID controller example: "
while true; do
    read -n 1 key
    if [[ $key == "s" || $key == "S" ]]; then
        echo ""
        break
    fi
done

# Step 2: Run the temperature control example
echo "Starting PID controller example..."
cargo run --package pidgeon --example debug_temperature_control --features=debugging &
CONTROLLER_PID=$!
echo "PID controller example started with PID: $CONTROLLER_PID"
echo ""

echo "All components have been started!"
echo "Open your browser to http://localhost:3000 to view the dashboard"
echo ""
echo "Press Ctrl+C to stop all components"

# Wait for both processes
# This will keep the script running until Ctrl+C is pressed
# or until either process terminates
wait 