# Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
# Initialization code that may require console input (password prompts, [y/n]
# confirmations, etc.) must go above this block; everything else may go below.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
  source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

export LANG='en_US.UTF-8'
export LANGUAGE='en_US:en'
export LC_ALL='en_US.UTF-8'
export TERM=xterm

##### Zsh/Oh-my-Zsh Configuration
export ZSH="/root/.oh-my-zsh"

ZSH_THEME="powerlevel10k/powerlevel10k"
plugins=(git zsh-autosuggestions zsh-completions )


source $ZSH/oh-my-zsh.sh
POWERLEVEL9K_SHORTEN_STRATEGY="truncate_to_last"
POWERLEVEL9K_LEFT_PROMPT_ELEMENTS=(user dir vcs status)
POWERLEVEL9K_RIGHT_PROMPT_ELEMENTS=()
POWERLEVEL9K_STATUS_OK=false
POWERLEVEL9K_STATUS_CROSS=true
export DISABLE_AUTO_TITLE=true
LC_NUMERIC="en_US.UTF-8"
source /opt/ros/humble/setup.zsh
source /usr/share/gazebo/setup.sh
alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"
alias cbuild="colcon build --symlink-install"
alias ssetup="source ./install/local_setup.zsh"
alias zenoh="export RMW_IMPLEMENTATION=rmw_zenoh_cpp"
alias cyclone="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
alias fastdds="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
autoload -U bashcompinit
bashcompinit
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

# To customize prompt, run `p10k configure` or edit ~/.p10k.zsh.
[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh
