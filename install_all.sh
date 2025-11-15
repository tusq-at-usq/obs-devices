

CWD="$(pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing all components..."

# Install PC setup
echo "Installing PC setup..."
./install_pc_setup.sh
# Install Certus
echo "Installing Certus..."
./install_certus.sh
# Install Encoders
echo "Installing Encoders..."
./install_encoders.sh
# Install Cameras
echo "Installing Cameras..."
./install_cameras.sh


uv pip install $SCRIPT_DIR

reboot_on_confirm() {
  read -p "Setup complete. Reboot required for permission to take effect. Reboot now? (y/n): " choice
  case "$choice" in 
    y|Y ) echo "Rebooting..."; sudo reboot;;
    n|N ) echo "Reboot cancelled. Please reboot later to apply all changes.";;
    * ) echo "Invalid input. Please enter y or n.";;
  esac
}
