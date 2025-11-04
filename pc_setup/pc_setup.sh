#!/bin/bash

build_python_venv() {
    if [ -d $HOME/.env ]
    then
        echo "Skipping virtual environment creation as $HOME/.env/obs/ already exists"
    else
        # Create Python virtual environment
        mkdir ~/.env
        sudo apt install -y python3-venv
        python3 -m venv ~/.env/obs
        cat >> ~/.bashrc<< EOF
        if [[ -f ~/.env/obs/bin/activate ]] ; then
            source ~/.env/obs/bin/activate
        fi 
EOF
    fi
    source ~/.env/obs/bin/activate
    pip install --upgrade pip


}


install_jetbrains_fonts() {
    JET_BRAINS_FONTS=$(ls ~/.local/share/fonts/JetBrains*Nerd* | wc -l)
    if [ $JET_BRAINS_FONTS -eq 0 ]
    then
        wget -P ~/.local/share/fonts https://github.com/ryanoasis/nerd-fonts/releases/download/v3.2.1/JetBrainsMono.zip \
        && cd ~/.local/share/fonts \
        && unzip JetBrainsMono.zip \
        && rm JetBrainsMono.zip \
        && sudo fc-cache -fv ~/.local/share/fonts
        dconf load /org/gnome/terminal/legacy/profiles:/ < ./gnome-terminal-profiles.dconf
    else
        echo "Skipping JetBrains Nerd font install as they already exist in $HOME/.local/share/fonts"
    fi
}

install_system_pkg_dependencies() {
    # Overheads
    sudo apt update -y && 
    sudo apt install -y curl build-essential python3-venv python3-pip chrony
}

install_python_packages() {
    pip install --upgrade pip
    pip install uv
    uv pip install ~/obs-utils/.
}

create_config_dir() {
    if [ ! -d $HOME/obs-config ]
    then
        mkdir $HOME/obs-config
        cp ./../tui/config.yaml $HOME/obs-config/tui_default_config.yaml
    else
        echo "$HOME/obs-config already exists, skipping creation"
    fi
}

install_nvim() {
    if [ -d /opt/nvim-linux-x86_64 ]
    then
        echo "Skipping nvim-linux64 install as it already exists in /opt/nvim-linux64"
    else
        # Install nvim for easy file editing
        curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.tar.gz
        sudo rm -rf /opt/nvim
        sudo tar -C /opt -xzf nvim-linux-x86_64.tar.gz
        export PATH="$PATH:/opt/nvim-linux-x86_64/bin"
        echo 'export PATH="$PATH:/opt/nvim-linux-x86_64/bin"' >> ~/.bashrc
    fi 
    # Copy nice nvim settings to .config directory
    sudo cp -r ./nvim ~/.config/ && nvim -c ':exe "normal iPress :q! <ENTER> when install finished. Note font will look odd until reboot"'

    if [ $(cat ~/.bashrc | grep -i 'alias vim' | wc -l) -eq 0 ]
    then
        echo "alias vim=nvim" >> ~/.bashrc
        echo "alias ll='ls -alF'" >> ~/.bashrc
        echo "alias la='ls -A'" >> ~/.bashrc
        echo "alias l='ls -CF'" >> ~/.bashrc
        echo "alias py=python3" >> ~/.bashrc
        echo "Added nvim alias to $HOME/.bashrc"
    else
        echo "Did not add nvim alias to $HOME/.bashrc"
    fi
}

install_vscode() {
  snap install --classic code
}


sudo apt update
sudo apt upgrade
build_python_venv
# Set timezone to UTC
sudo timedatectl set-timezone UTC
install_jetbrains_fonts
install_system_pkg_dependencies
install_python_packages
create_config_dir
install_nvim
install_vscode
