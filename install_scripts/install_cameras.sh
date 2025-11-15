
curl -L "https://downloads.alliedvision.com/VimbaX/VimbaX_Setup-2025-3-Linux64.tar.gz" -o "$HOME/Downloads/VimbaX_Setup-2025-3-Linux64.tar.gz"
mkdir -p ~/VimbaX
tar -xf ~/Downloads/VimbaX_Setup-2025-3-Linux64.tar.gz -C ~/VimbaX --strip-components=1
sudo ~/VimbaX/cti/Install_GenTL_Path.sh

echo "Camera installation complete."
