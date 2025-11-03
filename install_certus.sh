#!/bin/bash

# Install dependencies
sudo apt-get update
sudo apt-get install -y build-essential cmake libssl-dev libboost-all-dev
openjdk-11-jdk

# Download and set up Certus Mini Manager
curl -LO https://www.advancednavigation.com/downloads/latest/certus-mini-d/CertusMiniManager-7.5.zip
unzip CertusMiniManager-7.5.zip -d certus_manager.jar

# Set up Python packages
pip install -r requirements.txt

# Compile Certus C++ SDK interface
cd cpp/certus
make
cd ../../

