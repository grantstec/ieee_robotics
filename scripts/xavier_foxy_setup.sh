#!/bin/bash
# Xavier NX Optimization for ROS2 Foxy

echo "Setting up Xavier NX for ROS2 Foxy with GPU acceleration..."

# Maximum performance mode
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks   # Max clock speeds

# Create CUDA configuration directory if it doesn't exist
mkdir -p ~/.cuda

# Configure CUDA environment
cat > ~/.cuda/foxy_cuda_setup.sh << 'EOF'
#!/bin/bash
# CUDA setup for ROS2 Foxy on Xavier NX

# CUDA paths
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# OpenCV CUDA settings
export OPENCV_DNN_CUDA=ON

# ROS2 Foxy with CycloneDDS for better performance
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create CycloneDDS config directory if it doesn't exist
mkdir -p ~/cyclonedds_config

# Better CycloneDDS configuration for robotics
cat > ~/cyclonedds_config/cyclonedds.xml << 'XML_EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <Watermarks>
        <WhcHigh>8MB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
      <OutputFile>/tmp/cyclonedds.log</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
XML_EOF

export CYCLONEDDS_URI=~/cyclonedds_config/cyclonedds.xml

# Show environment status
echo "CUDA Environment:"
echo "CUDA Path: $PATH"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "RMW Implementation: $RMW_IMPLEMENTATION"

# Check CUDA availability with OpenCV
if python3 -c "import cv2; print('CUDA available:', cv2.cuda.getCudaEnabledDeviceCount() > 0)" 2>/dev/null; then
  echo "OpenCV CUDA configuration: OK"
else
  echo "OpenCV CUDA not detected, installing CUDA support..."
  sudo apt update
  sudo apt install -y libopencv-dev python3-opencv libopencv-cuda4.2
fi

# System monitoring tools
echo "Installing system monitoring tools..."
sudo apt install -y nvtop htop
EOF

# Make the script executable
chmod +x ~/.cuda/foxy_cuda_setup.sh

# Add to .bashrc
if ! grep -q "foxy_cuda_setup.sh" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# ROS2 Foxy with CUDA setup" >> ~/.bashrc
  echo "source ~/.cuda/foxy_cuda_setup.sh" >> ~/.bashrc
  echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
  echo "source ~/foxy_ws/install/setup.bash" >> ~/.bashrc
fi

# Run the script now
source ~/.cuda/foxy_cuda_setup.sh

echo "Xavier NX setup complete! Please restart your terminal."