
CWD="$(pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo apt install build-essential curl git file pkg-config swig \
       libcairo2-dev libnetpbm-dev netpbm libpng-dev libjpeg-dev \
       zlib1g-dev libbz2-dev libcfitsio-dev wcslib-dev \
       python3 python3-pip python3-dev \
       python3-numpy python3-scipy python3-pil python3-setuptools

# sudo apt install libnetpbm10-dev


cd "$SCRIPT_DIR"/../obs_astro/assets
wget http://astrometry.net/downloads/astrometry.net-latest.tar.gz
tar xvzf astrometry.net-latest.tar.gz
rm astrometry.net-latest.tar.gz
cd astrometry.net-0*

sudo apt install libnetpbm-dev
sudo ln -s /usr/include/netpbm /usr/local/include/netpbm
make reconfig
make
make py
make extra
sudo make install  # to put it in /usr/local/astrometry


if ! grep -q '/usr/local/astrometry/bin' ~/.bashrc; then
  echo 'export PATH="$PATH:/usr/local/astrometry/bin"' >> ~/.bashrc
  export PATH="$PATH:/usr/local/astrometry/bin"
fi

sudo cp "$SCRIPT_DIR"/assets/astrometry.cfg /usr/local/astrometry/etc/
sudo wget -P /usr/local/astrometry/data -np -r http://data.astrometry.net/4100/

